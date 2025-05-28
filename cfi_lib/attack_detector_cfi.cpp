// attack_detector_cfi.cpp
// 실제 공격을 탐지하는 CFI 구현

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <dlfcn.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <mutex>
#include <cxxabi.h>
#include <fstream>
#include <sstream>

#define LOG(fmt, ...) do { \
    fprintf(stderr, "[CFI] " fmt "\n", ##__VA_ARGS__); \
    fflush(stderr); \
} while(0)

#define SUCCESS(fmt, ...) do { \
    fprintf(stderr, "\033[0;32m[CFI-SUCCESS] " fmt "\033[0m\n", ##__VA_ARGS__); \
    fflush(stderr); \
} while(0)

#define ERROR(fmt, ...) do { \
    fprintf(stderr, "\033[0;31m[CFI-ERROR] " fmt "\033[0m\n", ##__VA_ARGS__); \
    fflush(stderr); \
} while(0)

#define WARNING(fmt, ...) do { \
    fprintf(stderr, "\033[0;33m[CFI-WARNING] " fmt "\033[0m\n", ##__VA_ARGS__); \
    fflush(stderr); \
} while(0)

// 전역 변수
static std::mutex g_mutex;
static std::unordered_map<void*, std::string> g_subscription_info;  // subscription -> topic name
static std::unordered_map<void*, void*> g_initial_callbacks;  // subscription -> initial callback
static std::unordered_map<void*, size_t> g_subscription_checksums;  // subscription -> checksum

// 통계
static size_t g_total_calls = 0;
static size_t g_violations = 0;
static size_t g_blocked_calls = 0;

// 원본 함수들
static void (*original_execute_subscription)(void*, void*) = nullptr;
static void* (*original_create_subscription)(void*, const void*, const void*, const void*) = nullptr;

// 환경 변수
static bool g_cfi_enabled = true;
static bool g_cfi_strict = false;

// demangle
std::string demangle(const char* name) {
    int status = 0;
    char* demangled = abi::__cxa_demangle(name, nullptr, nullptr, &status);
    std::string result = (status == 0 && demangled) ? demangled : name;
    if (demangled) free(demangled);
    return result;
}

// 메모리가 실행 가능한지 확인
bool is_executable_memory(void* addr) {
    std::ifstream maps("/proc/self/maps");
    std::string line;
    uintptr_t target = reinterpret_cast<uintptr_t>(addr);
    
    while (std::getline(maps, line)) {
        std::istringstream iss(line);
        std::string range, perms;
        iss >> range >> perms;
        
        size_t dash = range.find('-');
        if (dash != std::string::npos) {
            uintptr_t start = std::stoull(range.substr(0, dash), nullptr, 16);
            uintptr_t end = std::stoull(range.substr(dash + 1), nullptr, 16);
            
            if (target >= start && target < end) {
                return perms.find('x') != std::string::npos;
            }
        }
    }
    return false;
}

// Subscription에서 콜백 포인터 추출 (여러 위치 시도)
void* extract_callback_pointer(void* subscription) {
    unsigned char* bytes = (unsigned char*)subscription;
    
    // 일반적인 콜백 오프셋들 (실험적으로 발견된 값들)
    const size_t offsets[] = {48, 56, 64, 72, 80, 88, 96, 104, 112, 120, 128, 136, 144, 152};
    
    for (size_t offset : offsets) {
        void* potential = *(void**)(bytes + offset);
        
        if (potential && is_executable_memory(potential)) {
            Dl_info info;
            if (dladdr(potential, &info) && info.dli_sname) {
                std::string name = demangle(info.dli_sname);
                // 콜백 관련 심볼인지 확인
                if (name.find("operator()") != std::string::npos ||
                    name.find("callback") != std::string::npos ||
                    name.find("MinimalSubscriber") != std::string::npos) {
                    return potential;
                }
            }
        }
        
        // std::function 내부도 확인
        if (potential && (uintptr_t)potential > 0x1000 && (uintptr_t)potential < 0x800000000000) {
            unsigned char* func_bytes = (unsigned char*)potential;
            for (size_t i = 0; i < 32; i += 8) {
                void* func_ptr = *(void**)(func_bytes + i);
                if (func_ptr && is_executable_memory(func_ptr)) {
                    return func_ptr;
                }
            }
        }
    }
    
    return nullptr;
}

// Subscription 체크섬 계산
size_t calculate_subscription_checksum(void* subscription) {
    unsigned char* bytes = (unsigned char*)subscription;
    size_t checksum = 0;
    
    // 처음 256 바이트의 체크섬
    for (size_t i = 0; i < 256; i++) {
        checksum = checksum * 31 + bytes[i];
    }
    
    return checksum;
}

// std::string 추출 헬퍼
std::string extract_string(const void* str_ptr) {
    try {
        const char** data_ptr = (const char**)str_ptr;
        if (data_ptr && *data_ptr) {
            size_t* len_ptr = (size_t*)((char*)str_ptr + sizeof(char*));
            if (*len_ptr > 0 && *len_ptr < 1024) {
                return std::string(*data_ptr, *len_ptr);
            }
        }
    } catch (...) {}
    return "(unknown)";
}

// 생성자
__attribute__((constructor))
void cfi_init() {
    LOG("=== Attack Detector CFI Loaded ===");
    LOG("PID: %d", getpid());
    
    // 환경 변수
    const char* cfi_disable = getenv("ROS2_CFI_DISABLE");
    if (cfi_disable && strcmp(cfi_disable, "1") == 0) {
        g_cfi_enabled = false;
        WARNING("CFI is DISABLED");
    }
    
    const char* cfi_strict = getenv("ROS2_CFI_STRICT");
    if (cfi_strict && strcmp(cfi_strict, "1") == 0) {
        g_cfi_strict = true;
        LOG("Strict mode ENABLED");
    }
    
    // execute_subscription 후킹
    const char* exec_symbol = "_ZN6rclcpp8Executor20execute_subscriptionESt10shared_ptrINS_16SubscriptionBaseEE";
    original_execute_subscription = (void (*)(void*, void*))dlsym(RTLD_NEXT, exec_symbol);
    
    if (original_execute_subscription) {
        SUCCESS("Hooked execute_subscription");
    } else {
        ERROR("Failed to hook execute_subscription");
    }
    
    // create_subscription 후킹 시도 (선택적)
    /*const char* create_symbol = "_ZN6rclcpp15node_interfaces10NodeTopics19create_subscriptionERKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEERKNS_19SubscriptionFactoryERKNS_3QoSE";
    original_create_subscription = (void* (*)(void*, const void*, const void*, const void*))dlsym(RTLD_NEXT, create_symbol);
    
    if (original_create_subscription) {
        LOG("Also hooked create_subscription");
    }*/
}

// create_subscription 후킹 (topic 이름 저장)
extern "C" void* _ZN6rclcpp15node_interfaces10NodeTopics19create_subscriptionERKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEERKNS_19SubscriptionFactoryERKNS_3QoSE(
    void* this_ptr,
    const void* topic_name_ptr,
    const void* factory_ptr,
    const void* qos_ptr)
{
    std::string topic = extract_string(topic_name_ptr);
    
    void* subscription = nullptr;
    if (original_create_subscription) {
        subscription = original_create_subscription(this_ptr, topic_name_ptr, factory_ptr, qos_ptr);
        
        if (subscription && topic != "(unknown)") {
            std::lock_guard<std::mutex> lock(g_mutex);
            g_subscription_info[subscription] = topic;
            LOG("Registered subscription for topic: %s", topic.c_str());
        }
    }
    
    return subscription;
}

// execute_subscription 후킹 - 핵심 CFI 로직
extern "C" void _ZN6rclcpp8Executor20execute_subscriptionESt10shared_ptrINS_16SubscriptionBaseEE(
    void* executor_ptr, void* subscription_shared_ptr)
{
    g_total_calls++;
    
    // subscription 추출
    void* subscription = nullptr;
    if (subscription_shared_ptr) {
        subscription = *reinterpret_cast<void**>(subscription_shared_ptr);
    }
    
    if (!subscription) {
        if (original_execute_subscription) {
            original_execute_subscription(executor_ptr, subscription_shared_ptr);
        }
        return;
    }
    
    bool allow_execution = true;
    
    if (g_cfi_enabled) {
        std::lock_guard<std::mutex> lock(g_mutex);
        
        // Topic 이름 조회
        std::string topic = "(unknown)";
        auto topic_it = g_subscription_info.find(subscription);
        if (topic_it != g_subscription_info.end()) {
            topic = topic_it->second;
        }
        
        // 첫 호출인지 확인
        auto callback_it = g_initial_callbacks.find(subscription);
        if (callback_it == g_initial_callbacks.end()) {
            // 첫 호출 - 콜백과 체크섬 저장
            void* callback = extract_callback_pointer(subscription);
            size_t checksum = calculate_subscription_checksum(subscription);
            
            g_initial_callbacks[subscription] = callback;
            g_subscription_checksums[subscription] = checksum;
            
            if (callback) {
                Dl_info info;
                if (dladdr(callback, &info) && info.dli_sname) {
                    SUCCESS("Registered callback for %s: %p (%s)", 
                            topic.c_str(), callback, demangle(info.dli_sname).c_str());
                } else {
                    SUCCESS("Registered callback for %s: %p", topic.c_str(), callback);
                }
            }
        } else {
            // 이후 호출 - 변경 감지
            void* current_callback = extract_callback_pointer(subscription);
            size_t current_checksum = calculate_subscription_checksum(subscription);
            
            void* initial_callback = callback_it->second;
            size_t initial_checksum = g_subscription_checksums[subscription];
            
            // 콜백 변경 검사
            if (current_callback && initial_callback && current_callback != initial_callback) {
                g_violations++;
                ERROR("🚨 CALLBACK HIJACKING DETECTED! (Violation #%zu)", g_violations);
                ERROR("  Topic: %s", topic.c_str());
                ERROR("  Original callback: %p", initial_callback);
                ERROR("  Current callback: %p", current_callback);
                
                Dl_info info;
                if (dladdr(current_callback, &info) && info.dli_sname) {
                    ERROR("  Current symbol: %s", demangle(info.dli_sname).c_str());
                } else {
                    ERROR("  Current symbol: (unknown)");
                }
                
                if (!is_executable_memory(current_callback)) {
                    ERROR("  ⚠️  Current callback is NOT in executable memory!");
                }
                
                if (g_cfi_strict) {
                    ERROR("  ❌ BLOCKING EXECUTION (strict mode)");
                    allow_execution = false;
                    g_blocked_calls++;
                } else {
                    WARNING("  ⚠️  ALLOWING EXECUTION (non-strict mode)");
                }
            }
            
            // 체크섬 변경 검사 (더 민감한 검사)
            if (current_checksum != initial_checksum) {
                WARNING("Subscription object modified (checksum changed)");
                WARNING("  Topic: %s", topic.c_str());
                WARNING("  This could indicate memory corruption or attack");
            }
        }
    }
    
    // 원본 호출 또는 차단
    if (allow_execution && original_execute_subscription) {
        original_execute_subscription(executor_ptr, subscription_shared_ptr);
    } else if (!allow_execution) {
        ERROR("Execution BLOCKED by CFI");
        if (g_cfi_strict) {
            ERROR("FATAL: Terminating due to CFI violation");
            abort();
        }
    }
}

// 소멸자
__attribute__((destructor))
void cfi_fini() {
    LOG("\n=== CFI Protection Summary ===");
    LOG("Total callback executions: %zu", g_total_calls);
    LOG("CFI violations detected: %zu", g_violations);
    LOG("Blocked executions: %zu", g_blocked_calls);
    LOG("Monitored subscriptions: %zu", g_initial_callbacks.size());
    
    if (g_violations > 0) {
        ERROR("⚠️  CFI detected %zu potential attacks!", g_violations);
    } else if (g_total_calls > 0) {
        SUCCESS("✅ No CFI violations detected in %zu executions", g_total_calls);
    }
}
