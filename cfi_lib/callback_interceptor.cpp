// callback_interceptor.cpp
// ROS2 콜백 실행 시점을 인터셉트하는 CFI 라이브러리

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <dlfcn.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <iostream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <mutex>
#include <cxxabi.h>
#include <execinfo.h>
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

#define ERROR_LOG(fmt, ...) do { \
    fprintf(stderr, "\033[0;31m[CFI-ERROR] " fmt "\033[0m\n", ##__VA_ARGS__); \
    fflush(stderr); \
} while(0)

#define SUCCESS_LOG(fmt, ...) do { \
    fprintf(stderr, "\033[0;32m[CFI-SUCCESS] " fmt "\033[0m\n", ##__VA_ARGS__); \
    fflush(stderr); \
} while(0)

// 전역 변수
static std::mutex g_mutex;
static std::unordered_map<void*, std::string> g_subscription_info;  // subscription -> topic name
static std::unordered_set<void*> g_whitelist_callbacks;  // 합법적인 콜백 주소들
static std::unordered_map<void*, void*> g_subscription_to_callback;  // subscription -> callback

// 통계
static size_t g_total_callbacks = 0;
static size_t g_blocked_callbacks = 0;

// 원본 함수 포인터들
static void (*original_execute_subscription)(void*, void*) = nullptr;
static void* (*original_create_subscription)(void*, const void*, const void*, const void*) = nullptr;

// 환경 변수 확인
static bool g_cfi_enabled = true;
static bool g_cfi_strict_mode = false;  // strict mode에서는 화이트리스트에 없는 콜백은 차단

// demangle 헬퍼
std::string demangle(const char* name) {
    int status = 0;
    char* demangled = abi::__cxa_demangle(name, nullptr, nullptr, &status);
    std::string result = (status == 0 && demangled) ? demangled : name;
    if (demangled) free(demangled);
    return result;
}

// std::string 추출
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

// 메모리 주소가 실행 가능한 영역인지 확인
bool is_executable_address(void* addr) {
    std::ifstream maps("/proc/self/maps");
    std::string line;
    
    uintptr_t target = reinterpret_cast<uintptr_t>(addr);
    
    while (std::getline(maps, line)) {
        std::istringstream iss(line);
        std::string range, perms;
        iss >> range >> perms;
        
        // 주소 범위 파싱
        size_t dash = range.find('-');
        if (dash != std::string::npos) {
            uintptr_t start = std::stoull(range.substr(0, dash), nullptr, 16);
            uintptr_t end = std::stoull(range.substr(dash + 1), nullptr, 16);
            
            // 실행 권한 확인 (x가 있는지)
            if (target >= start && target < end) {
                return perms.find('x') != std::string::npos;
            }
        }
    }
    return false;
}

// 콜백 주소 추출 시도 (subscription 객체에서)
void* extract_callback_from_subscription(void* subscription) {
    // Subscription 객체 구조 분석
    // 일반적으로 콜백은 AnySubscriptionCallback 객체 내부에 있음
    // 여러 오프셋을 시도해봄
    
    void** ptr = reinterpret_cast<void**>(subscription);
    
    // 다양한 오프셋에서 함수 포인터처럼 보이는 것 찾기
    for (size_t offset = 8; offset < 256; offset += 8) {
        void* potential_callback = ptr[offset / sizeof(void*)];
        
        if (potential_callback && is_executable_address(potential_callback)) {
            Dl_info info;
            if (dladdr(potential_callback, &info)) {
                std::string name = info.dli_sname ? demangle(info.dli_sname) : "";
                // 콜백 함수의 일반적인 패턴
                if (name.find("operator()") != std::string::npos ||
                    name.find("callback") != std::string::npos ||
                    name.find("topic_callback") != std::string::npos) {
                    LOG("Found potential callback at offset %zu: %p (%s)", offset, potential_callback, name.c_str());
                    return potential_callback;
                }
            }
        }
    }
    
    return nullptr;
}

// 생성자
__attribute__((constructor))
void cfi_init() {
    LOG("=== ROS2 CFI Callback Interceptor Loaded ===");
    LOG("PID: %d", getpid());
    
    // 환경 변수 확인
    const char* cfi_disable = getenv("ROS2_CFI_DISABLE");
    if (cfi_disable && strcmp(cfi_disable, "1") == 0) {
        g_cfi_enabled = false;
        LOG("CFI is DISABLED by environment variable");
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
        SUCCESS_LOG("Found execute_subscription at %p", original_execute_subscription);
    } else {
        ERROR_LOG("Could not find execute_subscription");
    }
    
    // create_subscription 후킹
    const char* create_sub_symbol = "_ZN6rclcpp15node_interfaces10NodeTopics19create_subscriptionERKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEERKNS_19SubscriptionFactoryERKNS_3QoSE";
    original_create_subscription = (void* (*)(void*, const void*, const void*, const void*))dlsym(RTLD_NEXT, create_sub_symbol);
    
    if (original_create_subscription) {
        SUCCESS_LOG("Found create_subscription at %p", original_create_subscription);
    }
}

// create_subscription 후킹 - 콜백 등록 시점
extern "C" void* _ZN6rclcpp15node_interfaces10NodeTopics19create_subscriptionERKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEERKNS_19SubscriptionFactoryERKNS_3QoSE(
    void* this_ptr,
    const void* topic_name_ptr,
    const void* factory_ptr,
    const void* qos_ptr)
{
    std::string topic_name = extract_string(topic_name_ptr);
    LOG(">>> create_subscription for topic: %s", topic_name.c_str());
    
    // 원본 함수 호출
    void* subscription = nullptr;
    if (original_create_subscription) {
        subscription = original_create_subscription(this_ptr, topic_name_ptr, factory_ptr, qos_ptr);
        
        if (subscription) {
            // subscription 정보 저장
            {
                std::lock_guard<std::mutex> lock(g_mutex);
                g_subscription_info[subscription] = topic_name;
            }
            
            // 콜백 추출 시도
            void* callback = extract_callback_from_subscription(subscription);
            if (callback) {
                std::lock_guard<std::mutex> lock(g_mutex);
                g_whitelist_callbacks.insert(callback);
                g_subscription_to_callback[subscription] = callback;
                SUCCESS_LOG("Registered callback %p for topic %s", callback, topic_name.c_str());
            }
        }
    }
    
    return subscription;
}

// execute_subscription 후킹 - 콜백 실행 시점
extern "C" void _ZN6rclcpp8Executor20execute_subscriptionESt10shared_ptrINS_16SubscriptionBaseEE(
    void* executor_ptr, void* subscription_shared_ptr)
{
    g_total_callbacks++;
    
    // shared_ptr에서 실제 subscription 포인터 추출
    // shared_ptr의 첫 번째 멤버가 실제 포인터
    void* subscription = nullptr;
    if (subscription_shared_ptr) {
        subscription = *reinterpret_cast<void**>(subscription_shared_ptr);
    }
    
    // subscription 정보 조회
    std::string topic_name = "(unknown)";
    void* expected_callback = nullptr;
    {
        std::lock_guard<std::mutex> lock(g_mutex);
        auto it = g_subscription_info.find(subscription);
        if (it != g_subscription_info.end()) {
            topic_name = it->second;
        }
        
        auto cb_it = g_subscription_to_callback.find(subscription);
        if (cb_it != g_subscription_to_callback.end()) {
            expected_callback = cb_it->second;
        }
    }
    
    LOG(">>> execute_subscription: topic=%s, subscription=%p", topic_name.c_str(), subscription);
    
    // CFI 검증
    bool allow_execution = true;
    
    if (g_cfi_enabled && expected_callback) {
        // 현재 콜백 주소 추출 시도
        void* current_callback = extract_callback_from_subscription(subscription);
        
        if (current_callback && current_callback != expected_callback) {
            ERROR_LOG("CALLBACK MISMATCH DETECTED!");
            ERROR_LOG("  Topic: %s", topic_name.c_str());
            ERROR_LOG("  Expected callback: %p", expected_callback);
            ERROR_LOG("  Current callback: %p", current_callback);
            
            // 실행 가능한 주소인지 확인
            if (!is_executable_address(current_callback)) {
                ERROR_LOG("  Current callback is NOT in executable memory!");
            }
            
            g_blocked_callbacks++;
            
            if (g_cfi_strict_mode) {
                allow_execution = false;
                ERROR_LOG("  BLOCKING EXECUTION (strict mode)");
            } else {
                ERROR_LOG("  ALLOWING EXECUTION (non-strict mode)");
            }
        }
    }
    
    // 원본 함수 호출 또는 차단
    if (allow_execution && original_execute_subscription) {
        original_execute_subscription(executor_ptr, subscription_shared_ptr);
    } else if (!allow_execution) {
        ERROR_LOG("Callback execution BLOCKED by CFI");
        // 선택적: 여기서 프로그램을 종료하거나 예외를 발생시킬 수 있음
        if (g_cfi_strict_mode) {
            ERROR_LOG("FATAL: CFI violation in strict mode. Terminating.");
            abort();
        }
    }
}

// 소멸자
__attribute__((destructor))
void cfi_fini() {
    LOG("=== CFI Callback Interceptor Shutting Down ===");
    LOG("Statistics:");
    LOG("  Total callback executions: %zu", g_total_callbacks);
    LOG("  Blocked callbacks: %zu", g_blocked_callbacks);
    LOG("  Registered callbacks: %zu", g_whitelist_callbacks.size());
    LOG("  Tracked subscriptions: %zu", g_subscription_info.size());
    
    if (g_blocked_callbacks > 0) {
        ERROR_LOG("CFI prevented %zu potential attacks!", g_blocked_callbacks);
    }
}
