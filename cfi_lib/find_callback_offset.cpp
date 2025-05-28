// find_callback_offset.cpp
// Subscription 객체에서 콜백 함수 포인터의 오프셋을 찾는 도구

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <dlfcn.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <unordered_map>
#include <mutex>
#include <cxxabi.h>

#define LOG(fmt, ...) do { \
    fprintf(stderr, "[OFFSET-FINDER] " fmt "\n", ##__VA_ARGS__); \
    fflush(stderr); \
} while(0)

// 전역 변수
static std::mutex g_mutex;
static std::unordered_map<void*, void*> g_known_callbacks;  // subscription -> known callback

// 원본 함수
static void* (*original_create_subscription)(void*, const void*, const void*, const void*) = nullptr;

// demangle
std::string demangle(const char* name) {
    int status = 0;
    char* demangled = abi::__cxa_demangle(name, nullptr, nullptr, &status);
    std::string result = (status == 0 && demangled) ? demangled : name;
    if (demangled) free(demangled);
    return result;
}

// 알려진 콜백 함수 (subscriber.cpp의 topic_callback)
void known_topic_callback(const void* msg) {
    // 더미 함수 - 주소만 필요
}

// Factory 분석하여 콜백 찾기
void* find_callback_in_factory(const void* factory_ptr) {
    LOG("Analyzing factory at %p", factory_ptr);
    
    // const를 제거하고 작업
    unsigned char* bytes = (unsigned char*)factory_ptr;
    
    for (size_t i = 0; i < 512; i += sizeof(void*)) {
        void* potential = *(void**)(bytes + i);
        if (potential) {
            Dl_info info;
            if (dladdr(potential, &info) && info.dli_sname) {
                std::string name = demangle(info.dli_sname);
                if (name.find("topic_callback") != std::string::npos ||
                    name.find("operator()") != std::string::npos ||
                    (name.find("std_msgs") != std::string::npos && name.find("String") != std::string::npos)) {
                    LOG("  Found callback in factory at offset %zu: %p (%s)", 
                        i, potential, name.c_str());
                    return potential;
                }
            }
        }
    }
    return nullptr;
}

// Subscription에서 콜백 찾기
void find_callback_in_subscription(void* subscription, void* known_callback) {
    LOG("\nSearching for callback %p in subscription %p", known_callback, subscription);
    
    unsigned char* bytes = (unsigned char*)subscription;
    
    // 512 바이트까지 검색
    for (size_t offset = 0; offset < 512; offset += sizeof(void*)) {
        void* value = *(void**)(bytes + offset);
        
        if (value == known_callback) {
            LOG(">>> FOUND CALLBACK at offset %zu (0x%zx)", offset, offset);
            
            // 주변 메모리도 출력
            LOG("Context around callback:");
            for (int i = -2; i <= 2; i++) {
                size_t ctx_offset = offset + i * sizeof(void*);
                if (ctx_offset < 512) {
                    void* ctx_value = *(void**)(bytes + ctx_offset);
                    LOG("  [%+d] offset %zu: %p", i, ctx_offset, ctx_value);
                }
            }
            return;
        }
        
        // 포인터가 콜백을 가리키는 포인터인 경우도 확인
        if (value && (uintptr_t)value > 0x1000 && (uintptr_t)value < 0x800000000000) {
            try {
                void* indirect = *(void**)value;
                if (indirect == known_callback) {
                    LOG(">>> FOUND CALLBACK (indirect) at offset %zu -> %p -> %p", 
                        offset, value, indirect);
                }
            } catch (...) {
                // 무시
            }
        }
    }
    
    LOG("Callback not found in direct scan");
    
    // AnySubscriptionCallback 패턴 찾기
    LOG("\nLooking for AnySubscriptionCallback pattern...");
    for (size_t offset = 0; offset < 256; offset += 8) {
        void* value = *(void**)(bytes + offset);
        if (value && (uintptr_t)value > 0x1000 && (uintptr_t)value < 0x800000000000) {
            // std::function은 보통 2개의 포인터 (함수 포인터 + 캡처된 데이터)
            try {
                unsigned char* func_bytes = (unsigned char*)value;
                for (int i = 0; i < 128; i += 8) {
                    void* func_ptr = *(void**)(func_bytes + i);
                    if (func_ptr == known_callback) {
                        LOG(">>> FOUND CALLBACK in std::function at subscription offset %zu, function offset %d", 
                            offset, i);
                        LOG("    std::function object at: %p", value);
                        LOG("    Callback pointer at: %p", func_bytes + i);
                        return;
                    }
                }
            } catch (...) {}
        }
    }
}

// std::string 내용 추출
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
void finder_init() {
    LOG("=== Callback Offset Finder Loaded ===");
    
    const char* create_sub_symbol = "_ZN6rclcpp15node_interfaces10NodeTopics19create_subscriptionERKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEERKNS_19SubscriptionFactoryERKNS_3QoSE";
    original_create_subscription = (void* (*)(void*, const void*, const void*, const void*))dlsym(RTLD_NEXT, create_sub_symbol);
    
    if (original_create_subscription) {
        LOG("Found create_subscription");
    }
}

// create_subscription 후킹
extern "C" void* _ZN6rclcpp15node_interfaces10NodeTopics19create_subscriptionERKNSt7__cxx1112basic_stringIcSt11char_traitsIcESaIcEEERKNS_19SubscriptionFactoryERKNS_3QoSE(
    void* this_ptr,
    const void* topic_name_ptr,
    const void* factory_ptr,
    const void* qos_ptr)
{
    std::string topic = extract_string(topic_name_ptr);
    LOG("\n>>> create_subscription called for topic: %s", topic.c_str());
    
    // Factory에서 콜백 찾기
    void* callback_from_factory = find_callback_in_factory(factory_ptr);
    
    // 원본 호출
    void* subscription = nullptr;
    if (original_create_subscription) {
        subscription = original_create_subscription(this_ptr, topic_name_ptr, factory_ptr, qos_ptr);
        
        if (subscription && callback_from_factory) {
            // Subscription 객체에서 콜백 위치 찾기
            find_callback_in_subscription(subscription, callback_from_factory);
            
            // 메모리 덤프
            LOG("\nSubscription memory dump (first 256 bytes):");
            unsigned char* bytes = (unsigned char*)subscription;
            for (size_t i = 0; i < 256; i++) {
                if (i % 16 == 0) fprintf(stderr, "\n  %04zx: ", i);
                fprintf(stderr, "%02x ", bytes[i]);
            }
            fprintf(stderr, "\n");
        }
    }
    
    return subscription;
}

// 소멸자
__attribute__((destructor))
void finder_fini() {
    LOG("=== Offset Finder Unloading ===");
}
