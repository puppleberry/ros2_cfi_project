// simple_callback_detector.cpp
// 간단한 방법으로 콜백 감지하기

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

#define LOG(fmt, ...) do { \
    fprintf(stderr, "[DETECTOR] " fmt "\n", ##__VA_ARGS__); \
    fflush(stderr); \
} while(0)

#define SUCCESS(fmt, ...) do { \
    fprintf(stderr, "\033[0;32m[DETECTOR-SUCCESS] " fmt "\033[0m\n", ##__VA_ARGS__); \
    fflush(stderr); \
} while(0)

#define ERROR(fmt, ...) do { \
    fprintf(stderr, "\033[0;31m[DETECTOR-ERROR] " fmt "\033[0m\n", ##__VA_ARGS__); \
    fflush(stderr); \
} while(0)

// 전역 변수
static std::mutex g_mutex;
static std::unordered_map<void*, std::string> g_subscriptions;  // subscription -> info
static std::unordered_set<void*> g_detected_callbacks;
static int g_message_count = 0;

// 원본 함수들
static void (*original_execute_subscription)(void*, void*) = nullptr;
static void* (*original_create_subscription)(void*, const void*, const void*, const void*) = nullptr;

// demangle
std::string demangle(const char* name) {
    int status = 0;
    char* demangled = abi::__cxa_demangle(name, nullptr, nullptr, &status);
    std::string result = (status == 0 && demangled) ? demangled : name;
    if (demangled) free(demangled);
    return result;
}

// 백트레이스에서 콜백 함수 찾기
void detect_callback_from_backtrace() {
    void* buffer[30];
    int nptrs = backtrace(buffer, 30);
    
    LOG("Analyzing backtrace (depth=%d):", nptrs);
    
    for (int i = 0; i < nptrs; i++) {
        Dl_info info;
        if (dladdr(buffer[i], &info) && info.dli_sname) {
            std::string name = demangle(info.dli_sname);
            
            // 콜백 관련 패턴 찾기
            if (name.find("topic_callback") != std::string::npos ||
                name.find("subscription_callback") != std::string::npos ||
                (name.find("MinimalSubscriber") != std::string::npos && name.find("operator()") != std::string::npos) ||
                (name.find("std_msgs") != std::string::npos && name.find("String") != std::string::npos)) {
                
                SUCCESS("Found callback in backtrace[%d]: %p", i, buffer[i]);
                SUCCESS("  Function: %s", name.c_str());
                
                std::lock_guard<std::mutex> lock(g_mutex);
                g_detected_callbacks.insert(buffer[i]);
            }
        }
    }
}

// subscription 메모리 덤프
void dump_subscription_memory(void* subscription) {
    LOG("\nDumping subscription memory at %p:", subscription);
    unsigned char* bytes = (unsigned char*)subscription;
    
    // 처음 128 바이트만 출력
    for (size_t i = 0; i < 128; i++) {
        if (i % 16 == 0) fprintf(stderr, "\n  %04zx: ", i);
        fprintf(stderr, "%02x ", bytes[i]);
    }
    fprintf(stderr, "\n");
}

// 생성자
__attribute__((constructor))
void detector_init() {
    LOG("=== Simple Callback Detector Loaded ===");
    LOG("PID: %d", getpid());
    
    // execute_subscription 후킹
    const char* exec_sub_symbol = "_ZN6rclcpp8Executor20execute_subscriptionESt10shared_ptrINS_16SubscriptionBaseEE";
    original_execute_subscription = (void (*)(void*, void*))dlsym(RTLD_NEXT, exec_sub_symbol);
    
    if (original_execute_subscription) {
        SUCCESS("Found execute_subscription at %p", original_execute_subscription);
    } else {
        ERROR("Could not find execute_subscription");
    }
    
    // create_subscription도 시도 (simple 버전)
    void* handle = dlopen("/opt/ros/humble/lib/librclcpp.so", RTLD_LAZY);
    if (handle) {
        // NodeTopics 버전 찾기
        const char* symbols[] = {
            "_ZN6rclcpp15node_interfaces10NodeTopics19create_subscriptionERKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEERKNS_19SubscriptionFactoryERKNS_3QoSE",
            "_ZN6rclcpp4Node19create_subscriptionIN10std_msgs3msg7String_ISaIcEEESt8functionIFvSt10shared_ptrIT_EEEJEEENS_16SubscriptionBaseIPtrISA_TnEENSt7__cxx1112basic_stringIcS5_S6_EERKNSI_3QoSESC_RKNS_19SubscriptionOptionsE",
            nullptr
        };
        
        for (const char** sym = symbols; *sym; sym++) {
            void* addr = dlsym(handle, *sym);
            if (addr) {
                LOG("Found symbol: %s", demangle(*sym).c_str());
                break;
            }
        }
    }
}

// execute_subscription 후킹
extern "C" void _ZN6rclcpp8Executor20execute_subscriptionESt10shared_ptrINS_16SubscriptionBaseEE(
    void* executor_ptr, void* subscription_shared_ptr)
{
    g_message_count++;
    
    // subscription 포인터 추출
    void* subscription = nullptr;
    if (subscription_shared_ptr) {
        subscription = *reinterpret_cast<void**>(subscription_shared_ptr);
    }
    
    LOG("\n>>> Message #%d received! execute_subscription called", g_message_count);
    LOG("  Executor: %p", executor_ptr);
    LOG("  Subscription: %p", subscription);
    
    // 첫 번째 메시지에서만 상세 분석
    if (g_message_count == 1 && subscription) {
        dump_subscription_memory(subscription);
    }
    
    // 백트레이스 분석은 원본 함수 호출 전에
    LOG("Analyzing call stack BEFORE calling original...");
    
    // 원본 함수 호출
    if (original_execute_subscription) {
        // 원본 함수 호출 중에 백트레이스 분석
        LOG("Calling original execute_subscription...");
        original_execute_subscription(executor_ptr, subscription_shared_ptr);
        LOG("Original function returned");
        
        // 호출 후에도 백트레이스 확인
        detect_callback_from_backtrace();
    }
}

// 소멸자
__attribute__((destructor))
void detector_fini() {
    LOG("\n=== Detector Summary ===");
    LOG("Total messages processed: %d", g_message_count);
    LOG("Unique callbacks detected: %zu", g_detected_callbacks.size());
    
    if (!g_detected_callbacks.empty()) {
        SUCCESS("\nDetected callbacks:");
        for (void* cb : g_detected_callbacks) {
            Dl_info info;
            if (dladdr(cb, &info) && info.dli_sname) {
                SUCCESS("  %p: %s", cb, demangle(info.dli_sname).c_str());
            } else {
                SUCCESS("  %p: (unknown)", cb);
            }
        }
    } else {
        ERROR("No callbacks detected!");
    }
}
