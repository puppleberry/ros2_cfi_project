// simple_hook_test.cpp
// 간단한 버전의 ROS2 CFI 라이브러리 - 테스트용

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

// 간단한 로그 매크로
#define LOG(fmt, ...) do { \
    fprintf(stderr, "[CFI] " fmt "\n", ##__VA_ARGS__); \
    fflush(stderr); \
} while(0)

// 전역 변수
static std::mutex g_mutex;
static std::unordered_set<void*> g_callbacks;
static bool g_initialized = false;

// 원본 함수 포인터 - void*로 단순화
typedef void* (*generic_create_subscription_fn)(void*, void*, void*, void*);
static generic_create_subscription_fn original_create_subscription = nullptr;

// 생성자 - 프로그램 시작 시 자동 실행
__attribute__((constructor))
void cfi_init() {
    LOG("=== Simple ROS2 CFI Library Loaded ===");
    LOG("PID: %d", getpid());
    
    // create_subscription 함수 찾기
    const char* func_name = "_ZN6rclcpp15node_interfaces10NodeTopics19create_subscriptionERKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEERKNS_19SubscriptionFactoryERKNS_3QoSE";
    
    original_create_subscription = (generic_create_subscription_fn)dlsym(RTLD_NEXT, func_name);
    
    if (original_create_subscription) {
        LOG("Found create_subscription at %p", original_create_subscription);
    } else {
        LOG("WARNING: Could not find create_subscription: %s", dlerror());
        
        // 다른 가능한 심볼들도 시도
        void* handle = dlopen("/opt/ros/humble/lib/librclcpp.so", RTLD_LAZY);
        if (handle) {
            original_create_subscription = (generic_create_subscription_fn)dlsym(handle, func_name);
            if (original_create_subscription) {
                LOG("Found create_subscription in librclcpp.so at %p", original_create_subscription);
            }
        }
    }
    
    g_initialized = true;
}

// 소멸자
__attribute__((destructor))
void cfi_fini() {
    LOG("=== CFI Library Unloading ===");
    LOG("Total callbacks tracked: %zu", g_callbacks.size());
}

// 후킹된 create_subscription - C++ name mangling 사용
extern "C" void* 
_ZN6rclcpp15node_interfaces10NodeTopics19create_subscriptionERKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEERKNS_19SubscriptionFactoryERKNS_3QoSE(
    void* this_ptr,
    void* topic_name_ptr,
    void* factory_ptr,
    void* qos_ptr)
{
    LOG(">>> HOOKED: create_subscription called");
    LOG("    This: %p", this_ptr);
    LOG("    Topic name ptr: %p", topic_name_ptr);
    LOG("    Factory ptr: %p", factory_ptr);
    LOG("    QoS ptr: %p", qos_ptr);
    
    // std::string의 c_str()을 얻기 위한 시도
    // std::string 구조를 가정 (대부분의 구현에서 첫 번째 멤버가 포인터)
    const char** str_ptr = (const char**)topic_name_ptr;
    if (str_ptr && *str_ptr) {
        LOG("    Topic: %s", *str_ptr);
    }
    
    // Factory의 첫 몇 바이트를 16진수로 출력
    if (factory_ptr) {
        const unsigned char* factory_bytes = reinterpret_cast<const unsigned char*>(factory_ptr);
        fprintf(stderr, "    Factory content: ");
        for (size_t i = 0; i < 32; ++i) {
            fprintf(stderr, "%02x ", factory_bytes[i]);
        }
        fprintf(stderr, "\n");
    }
    
    // 원본 함수 호출
    if (original_create_subscription) {
        LOG("    Calling original function...");
        void* result = original_create_subscription(this_ptr, topic_name_ptr, factory_ptr, qos_ptr);
        LOG("    Subscription created: %p", result);
        
        // 콜백 추적 (간단한 버전)
        if (result) {
            std::lock_guard<std::mutex> lock(g_mutex);
            g_callbacks.insert(result);
        }
        
        return result;
    } else {
        LOG("ERROR: Original function is NULL!");
        abort();
    }
}
