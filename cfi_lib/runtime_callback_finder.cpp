// runtime_callback_finder.cpp
// execute_subscription 시점에서 콜백 찾기

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <dlfcn.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <iostream>
#include <unordered_set>
#include <mutex>
#include <cxxabi.h>
#include <fstream>
#include <sstream>

#define LOG(fmt, ...) do { \
    fprintf(stderr, "[RUNTIME-FIND] " fmt "\n", ##__VA_ARGS__); \
    fflush(stderr); \
} while(0)

// 전역 변수
static std::mutex g_mutex;
static std::unordered_set<void*> g_found_callbacks;
static void (*original_execute_subscription)(void*, void*) = nullptr;

// demangle
std::string demangle(const char* name) {
    int status = 0;
    char* demangled = abi::__cxa_demangle(name, nullptr, nullptr, &status);
    std::string result = (status == 0 && demangled) ? demangled : name;
    if (demangled) free(demangled);
    return result;
}

// 메모리가 실행 가능한지 확인
bool is_executable(void* addr) {
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

// Subscription 객체에서 가능한 콜백 찾기
void analyze_subscription(void* subscription) {
    LOG("Analyzing subscription at %p", subscription);
    
    unsigned char* bytes = (unsigned char*)subscription;
    std::unordered_set<void*> candidates;
    
    // 처음 512 바이트 스캔
    for (size_t offset = 0; offset < 512; offset += 8) {
        void* ptr = *(void**)(bytes + offset);
        
        if (ptr && is_executable(ptr)) {
            Dl_info info;
            if (dladdr(ptr, &info)) {
                std::string name = info.dli_sname ? demangle(info.dli_sname) : "";
                
                // 콜백 관련 심볼 필터링
                if (name.find("topic_callback") != std::string::npos ||
                    name.find("operator()") != std::string::npos ||
                    name.find("subscription_callback") != std::string::npos ||
                    (name.find("MinimalSubscriber") != std::string::npos && name.find("operator()") != std::string::npos)) {
                    
                    LOG("  Found callback candidate at offset %zu: %p", offset, ptr);
                    LOG("    Symbol: %s", name.c_str());
                    candidates.insert(ptr);
                }
            }
        }
        
        // 간접 참조도 확인
        if (ptr && (uintptr_t)ptr > 0x1000 && (uintptr_t)ptr < 0x800000000000) {
            try {
                void* indirect = *(void**)ptr;
                if (indirect && is_executable(indirect)) {
                    Dl_info info;
                    if (dladdr(indirect, &info)) {
                        std::string name = info.dli_sname ? demangle(info.dli_sname) : "";
                        if (name.find("topic_callback") != std::string::npos ||
                            name.find("operator()") != std::string::npos) {
                            LOG("  Found indirect callback at offset %zu -> %p -> %p", offset, ptr, indirect);
                            LOG("    Symbol: %s", name.c_str());
                            candidates.insert(indirect);
                        }
                    }
                }
            } catch (...) {}
        }
    }
    
    // AnySubscriptionCallback 패턴 찾기
    for (size_t offset = 0; offset < 256; offset += 8) {
        void* obj = *(void**)(bytes + offset);
        if (obj && (uintptr_t)obj > 0x1000 && (uintptr_t)obj < 0x800000000000) {
            try {
                // std::function 객체 내부 탐색
                unsigned char* func_bytes = (unsigned char*)obj;
                for (size_t i = 0; i < 64; i += 8) {
                    void* func_ptr = *(void**)(func_bytes + i);
                    if (func_ptr && is_executable(func_ptr)) {
                        Dl_info info;
                        if (dladdr(func_ptr, &info) && info.dli_sname) {
                            std::string name = demangle(info.dli_sname);
                            if (name.find("topic_callback") != std::string::npos ||
                                name.find("operator()") != std::string::npos) {
                                LOG("  Found callback in std::function at subscription offset %zu, function offset %zu", offset, i);
                                LOG("    Callback: %p (%s)", func_ptr, name.c_str());
                                candidates.insert(func_ptr);
                            }
                        }
                    }
                }
            } catch (...) {}
        }
    }
    
    // 결과 저장
    if (!candidates.empty()) {
        std::lock_guard<std::mutex> lock(g_mutex);
        for (void* cb : candidates) {
            g_found_callbacks.insert(cb);
        }
        LOG("  Total unique callbacks found so far: %zu", g_found_callbacks.size());
    }
}

// 생성자
__attribute__((constructor))
void finder_init() {
    LOG("=== Runtime Callback Finder Loaded ===");
    
    const char* exec_sub_symbol = "_ZN6rclcpp8Executor20execute_subscriptionESt10shared_ptrINS_16SubscriptionBaseEE";
    original_execute_subscription = (void (*)(void*, void*))dlsym(RTLD_NEXT, exec_sub_symbol);
    
    if (original_execute_subscription) {
        LOG("Found execute_subscription");
    }
}

// execute_subscription 후킹
extern "C" void _ZN6rclcpp8Executor20execute_subscriptionESt10shared_ptrINS_16SubscriptionBaseEE(
    void* executor_ptr, void* subscription_shared_ptr)
{
    static int call_count = 0;
    call_count++;
    
    // shared_ptr에서 실제 포인터 추출
    void* subscription = nullptr;
    if (subscription_shared_ptr) {
        subscription = *reinterpret_cast<void**>(subscription_shared_ptr);
    }
    
    LOG("\n>>> execute_subscription call #%d", call_count);
    LOG("  Subscription: %p", subscription);
    
    if (subscription) {
        analyze_subscription(subscription);
    }
    
    // 원본 함수 호출
    if (original_execute_subscription) {
        original_execute_subscription(executor_ptr, subscription_shared_ptr);
    }
}

// 소멸자
__attribute__((destructor))
void finder_fini() {
    LOG("\n=== Runtime Finder Results ===");
    LOG("Total unique callbacks found: %zu", g_found_callbacks.size());
    
    std::lock_guard<std::mutex> lock(g_mutex);
    for (void* cb : g_found_callbacks) {
        Dl_info info;
        if (dladdr(cb, &info) && info.dli_sname) {
            LOG("  Callback: %p (%s)", cb, demangle(info.dli_sname).c_str());
        } else {
            LOG("  Callback: %p (unknown)", cb);
        }
    }
}
