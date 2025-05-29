// robust_hooking.cpp
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <dlfcn.h>
#include <stdio.h>
#include <string.h>

// 여러 방법으로 원본 함수 찾기
static void* find_original_function() {
    const char* symbol = "_ZN6rclcpp8Executor20execute_subscriptionESt10shared_ptrINS_16SubscriptionBaseEE";
    void* func = nullptr;
    
    // 1. RTLD_NEXT 시도
    func = dlsym(RTLD_NEXT, symbol);
    if (func) {
        printf("[HOOK] Found via RTLD_NEXT: %p\n", func);
        return func;
    }
    
    // 2. RTLD_DEFAULT에서 찾고 우리 것이 아닌지 확인
    func = dlsym(RTLD_DEFAULT, symbol);
    void* our_func = dlsym(RTLD_DEFAULT, symbol);
    Dl_info info;
    if (func && dladdr(func, &info)) {
        if (!strstr(info.dli_fname, "libday5")) {
            printf("[HOOK] Found via RTLD_DEFAULT: %p in %s\n", func, info.dli_fname);
            return func;
        }
    }
    
    // 3. 직접 librclcpp.so 열기
    void* handle = dlopen("librclcpp.so", RTLD_NOW | RTLD_GLOBAL);
    if (handle) {
        func = dlsym(handle, symbol);
        if (func) {
            printf("[HOOK] Found via direct dlopen: %p\n", func);
            return func;
        }
    }
    
    return nullptr;
}

// 후킹 함수
extern "C" void _ZN6rclcpp8Executor20execute_subscriptionESt10shared_ptrINS_16SubscriptionBaseEE(
    void* executor_ptr, void* subscription_ptr) 
{
    static void (*original)(void*, void*) = nullptr;
    static int call_count = 0;
    
    if (!original) {
        original = (void(*)(void*, void*))find_original_function();
        if (!original) {
            printf("[HOOK] FATAL: Cannot find original function!\n");
            return;
        }
    }
    
    printf("[HOOK] Intercepted call #%d\n", ++call_count);
    
    if (original) {
        original(executor_ptr, subscription_ptr);
    }
}
