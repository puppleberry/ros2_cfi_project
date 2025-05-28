// final_callback_hook.cpp
// 최종 CFI 구현 - 콜백 후킹 및 검증

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
#include <execinfo.h>
#include <signal.h>
#include <fstream>
#include <sstream>
#include <chrono>

#define LOG(fmt, ...) do { \
    auto now = std::chrono::system_clock::now(); \
    auto time_t = std::chrono::system_clock::to_time_t(now); \
    char time_buf[32]; \
    strftime(time_buf, sizeof(time_buf), "%H:%M:%S", localtime(&time_t)); \
    fprintf(stderr, "[%s][CFI] " fmt "\n", time_buf, ##__VA_ARGS__); \
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
static std::unordered_map<void*, std::string> g_subscription_info;  // subscription -> topic
static std::unordered_set<void*> g_whitelist_callbacks;  // 합법적인 콜백들
static std::unordered_map<void*, void*> g_subscription_callbacks;  // subscription -> 원래 콜백

// 통계
static size_t g_total_executions = 0;
static size_t g_blocked_executions = 0;
static size_t g_callback_changes_detected = 0;

// 환경 변수
static bool g_cfi_enabled = true;
static bool g_cfi_strict_mode = false;

// 원본 함수들
static void (*original_execute_subscription)(void*, void*) = nullptr;

// demangle
std::string demangle(const char* name) {
    int status = 0;
    char* demangled = abi::__cxa_demangle(name, nullptr, nullptr, &status);
    std::string result = (status == 0 && demangled) ? demangled : name;
    if (demangled) free(demangled);
    return result;
}

// 메모리 영역이 실행 가능한지 확인
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

// Subscription에서 콜백 추출 (간단한 휴리스틱)
void* extract_callback_simple(void* subscription) {
    // AnySubscriptionCallback은 보통 subscription 객체의 특정 오프셋에 있음
    // 일반적인 오프셋들을 시도
    const size_t common_offsets[] = {48, 56, 64, 72, 80, 88, 96, 104, 112, 120, 128};
    
    unsigned char* bytes = (unsigned char*)subscription;
    
    for (size_t offset : common_offsets) {
        void* potential = *(void**)(bytes + offset);
        
        // 포인터가 std::function 객체를 가리킬 수 있음
        if (potential && (uintptr_t)potential > 0x1000) {
            // std::function 내부에서 실제 함수 포인터 찾기
            unsigned char* func_bytes = (unsigned char*)potential;
            for (size_t i = 0; i < 32; i += 8) {
                void* func_ptr = *(void**)(func_bytes + i);
                if (func_ptr && is_executable_memory(func_ptr)) {
                    Dl_info info;
                    if (dladdr(func_ptr, &info) && info.dli_sname) {
                        std::string name = demangle(info.dli_sname);
                        if (name.find("operator()") != std::string::npos ||
                            name.find("callback") != std::string::npos) {
                            return func_ptr;
                        }
                    }
                }
            }
        }
    }
    
    return nullptr;
}

// 생성자
__attribute__((constructor))
void cfi_init() {
    LOG("=== ROS2 CFI Protection Initialized ===");
    LOG("PID: %d", getpid());
    
    // 환경 변수 확인
    const char* cfi_disable = getenv("ROS2_CFI_DISABLE");
    if (cfi_disable && strcmp(cfi_disable, "1") == 0) {
        g_cfi_enabled = false;
        WARNING("CFI is DISABLED by environment variable");
    }
    
    const char* cfi_strict = getenv("ROS2_CFI_STRICT");
    if (cfi_strict && strcmp(cfi_strict, "1") == 0) {
        g_cfi_strict_mode = true;
        LOG("CFI strict mode ENABLED");
    }
    
    // execute_subscription 후킹
    const char* exec_sub_symbol = "_ZN6rclcpp8Executor20execute_subscriptionESt10shared_ptrINS_16SubscriptionBaseEE";
    original_execute_subscription = (void (*)(void*, void*))dlsym(RTLD_NEXT, exec_sub_symbol);
    
    if (original_execute_subscription) {
        SUCCESS("Hooked execute_subscription at %p", original_execute_subscription);
    } else {
        ERROR("Failed to hook execute_subscription");
    }
}

// execute_subscription 후킹 - 핵심 CFI 로직
extern "C" void _ZN6rclcpp8Executor20execute_subscriptionESt10shared_ptrINS_16SubscriptionBaseEE(
    void* executor_ptr, void* subscription_shared_ptr)
{
    g_total_executions++;
    
    // subscription 포인터 추출
    void* subscription = nullptr;
    if (subscription_shared_ptr) {
        subscription = *reinterpret_cast<void**>(subscription_shared_ptr);
    }
    
    // Subscription 정보 조회
    std::string topic_name = "(unknown)";
    void* registered_callback = nullptr;
    
    {
        std::lock_guard<std::mutex> lock(g_mutex);
        auto info_it = g_subscription_info.find(subscription);
        if (info_it != g_subscription_info.end()) {
            topic_name = info_it->second;
        }
        
        auto cb_it = g_subscription_callbacks.find(subscription);
        if (cb_it != g_subscription_callbacks.end()) {
            registered_callback = cb_it->second;
        }
    }
    
    // 현재 콜백 추출
    void* current_callback = extract_callback_simple(subscription);
    
    // 첫 실행이면 콜백 등록
    if (!registered_callback && current_callback) {
        std::lock_guard<std::mutex> lock(g_mutex);
        g_subscription_callbacks[subscription] = current_callback;
        g_whitelist_callbacks.insert(current_callback);
        registered_callback = current_callback;
        
        Dl_info info;
        if (dladdr(current_callback, &info) && info.dli_sname) {
            SUCCESS("Registered callback for topic %s: %p (%s)", 
                    topic_name.c_str(), current_callback, demangle(info.dli_sname).c_str());
        } else {
            SUCCESS("Registered callback for topic %s: %p", topic_name.c_str(), current_callback);
        }
    }
    
    // CFI 검증
    bool allow_execution = true;
    
    if (g_cfi_enabled && registered_callback && current_callback) {
        if (current_callback != registered_callback) {
            g_callback_changes_detected++;
            
            ERROR("🚨 CFI VIOLATION DETECTED!");
            ERROR("  Topic: %s", topic_name.c_str());
            ERROR("  Expected callback: %p", registered_callback);
            ERROR("  Current callback: %p", current_callback);
            
            Dl_info info;
            if (dladdr(current_callback, &info) && info.dli_sname) {
                ERROR("  Current symbol: %s", demangle(info.dli_sname).c_str());
            }
            
            if (!is_executable_memory(current_callback)) {
                ERROR("  ⚠️  Current callback is NOT in executable memory!");
            }
            
            if (g_cfi_strict_mode) {
                allow_execution = false;
                g_blocked_executions++;
                ERROR("  ❌ BLOCKING EXECUTION (strict mode)");
            } else {
                WARNING("  ⚠️  ALLOWING EXECUTION (non-strict mode)");
            }
        }
    }
    
    // 원본 함수 호출 또는 차단
    if (allow_execution && original_execute_subscription) {
        original_execute_subscription(executor_ptr, subscription_shared_ptr);
    } else if (!allow_execution) {
        ERROR("Callback execution BLOCKED by CFI");
        if (g_cfi_strict_mode) {
            ERROR("FATAL: CFI violation in strict mode. Terminating.");
            abort();
        }
    }
}

// Subscription 정보 등록 (다른 라이브러리에서 호출할 수 있도록)
extern "C" void ros2_cfi_register_subscription(void* subscription, const char* topic) {
    if (subscription && topic) {
        std::lock_guard<std::mutex> lock(g_mutex);
        g_subscription_info[subscription] = topic;
        LOG("Registered subscription %p for topic %s", subscription, topic);
    }
}

// 소멸자
__attribute__((destructor))
void cfi_fini() {
    LOG("\n=== CFI Protection Summary ===");
    LOG("Total callback executions: %zu", g_total_executions);
    LOG("Blocked executions: %zu", g_blocked_executions);
    LOG("Callback changes detected: %zu", g_callback_changes_detected);
    LOG("Registered callbacks: %zu", g_whitelist_callbacks.size());
    
    if (g_callback_changes_detected > 0) {
        ERROR("⚠️  CFI detected %zu potential attacks!", g_callback_changes_detected);
    } else if (g_total_executions > 0) {
        SUCCESS("✓ No CFI violations detected in %zu executions", g_total_executions);
    }
}
