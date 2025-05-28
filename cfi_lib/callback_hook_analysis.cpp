// callback_hook_analysis.cpp
// ROS2 콜백 호출 경로 분석 및 후킹 지점 찾기

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
#include <unordered_set>
#include <mutex>
#include <cxxabi.h>
#include <execinfo.h>

#define LOG(fmt, ...) do { \
    fprintf(stderr, "[ANALYSIS] " fmt "\n", ##__VA_ARGS__); \
    fflush(stderr); \
} while(0)

// 전역 변수
static bool g_initialized = false;
static std::mutex g_mutex;

// 원본 함수들
static void* (*original_handle_message)(void*, void*, void*) = nullptr;
static void* (*original_execute_impl)(void*, void*) = nullptr;

// demangle 헬퍼
std::string demangle(const char* name) {
    int status = 0;
    char* demangled = abi::__cxa_demangle(name, nullptr, nullptr, &status);
    std::string result = (status == 0 && demangled) ? demangled : name;
    if (demangled) free(demangled);
    return result;
}

// 스택 트레이스 출력
void print_stack_trace() {
    void* buffer[20];
    int nptrs = backtrace(buffer, 20);
    char** strings = backtrace_symbols(buffer, nptrs);
    
    LOG("=== Stack Trace ===");
    for (int i = 0; i < nptrs; i++) {
        // 심볼에서 함수 이름 추출
        char* mangled_name = nullptr;
        char* offset_begin = nullptr;
        char* offset_end = nullptr;
        
        for (char *p = strings[i]; *p; ++p) {
            if (*p == '(') {
                mangled_name = p + 1;
            } else if (*p == '+') {
                offset_begin = p;
            } else if (*p == ')') {
                offset_end = p;
                break;
            }
        }
        
        if (mangled_name && offset_begin && offset_end) {
            *offset_begin = '\0';
            std::string demangled = demangle(mangled_name);
            LOG("  [%d] %s", i, demangled.c_str());
        } else {
            LOG("  [%d] %s", i, strings[i]);
        }
    }
    free(strings);
}

// 생성자
__attribute__((constructor))
void analysis_init() {
    LOG("=== Callback Analysis Library Loaded ===");
    LOG("PID: %d", getpid());
    
    // 다양한 후킹 대상 찾기
    const char* symbols[] = {
        // SubscriptionBase 관련
        "_ZN6rclcpp16SubscriptionBase13handle_messageESt10shared_ptrIN3rcl3msg14MessageInfoPtrEEOS2_IvE",
        "_ZN6rclcpp16SubscriptionBase12execute_implESt10shared_ptrIvE",
        
        // AnySubscriptionCallback 관련
        "_ZN6rclcpp21AnySubscriptionCallback8dispatchESt10shared_ptrI10std_msgs3msg6StringISaIcEEERKN3rcl11MessageInfoE",
        "_ZN6rclcpp21AnySubscriptionCallback4callESt10shared_ptrI10std_msgs3msg6StringISaIcEEERKN3rcl11MessageInfoE",
        
        // Executor 관련
        "_ZN6rclcpp8Executor20execute_subscriptionESt10shared_ptrINS_16SubscriptionBaseEE",
        
        nullptr
    };
    
    for (const char** sym = symbols; *sym; sym++) {
        void* addr = dlsym(RTLD_NEXT, *sym);
        if (addr) {
            LOG("Found symbol: %s", demangle(*sym).c_str());
            LOG("  Address: %p", addr);
        }
    }
    
    // handle_message 후킹 시도
    original_handle_message = (void* (*)(void*, void*, void*))dlsym(RTLD_NEXT, 
        "_ZN6rclcpp16SubscriptionBase13handle_messageESt10shared_ptrIN3rcl3msg14MessageInfoPtrEEOS2_IvE");
    
    if (original_handle_message) {
        LOG("Successfully found handle_message for hooking");
    }
    
    g_initialized = true;
}

// handle_message 후킹
extern "C" void* _ZN6rclcpp16SubscriptionBase13handle_messageESt10shared_ptrIN3rcl3msg14MessageInfoPtrEEOS2_IvE(
    void* this_ptr, void* message_info, void* message)
{
    LOG(">>> HOOKED: SubscriptionBase::handle_message");
    LOG("    this: %p", this_ptr);
    LOG("    message_info: %p", message_info);
    LOG("    message: %p", message);
    
    // 스택 트레이스로 호출 경로 확인
    print_stack_trace();
    
    // RTTI로 타입 정보 얻기 시도
    try {
        const std::type_info& ti = typeid(*reinterpret_cast<void**>(this_ptr));
        LOG("    Type: %s", demangle(ti.name()).c_str());
    } catch (...) {
        LOG("    Type: (unknown)");
    }
    
    // 원본 함수 호출
    if (original_handle_message) {
        LOG("    Calling original handle_message...");
        void* result = original_handle_message(this_ptr, message_info, message);
        LOG("    handle_message completed");
        return result;
    }
    
    return nullptr;
}

// execute_subscription 후킹 (Executor가 subscription을 실행할 때)
extern "C" void _ZN6rclcpp8Executor20execute_subscriptionESt10shared_ptrINS_16SubscriptionBaseEE(
    void* this_ptr, void* subscription_ptr)
{
    LOG(">>> HOOKED: Executor::execute_subscription");
    LOG("    executor: %p", this_ptr);
    LOG("    subscription: %p", subscription_ptr);
    
    // 여기서 콜백이 호출될 예정
    LOG("    Callback is about to be called...");
    
    // TODO: 원본 함수 호출
}

// 소멸자
__attribute__((destructor))
void analysis_fini() {
    LOG("=== Analysis Library Unloading ===");
}
