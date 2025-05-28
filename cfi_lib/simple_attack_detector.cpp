// simple_attack_detector.cpp
// 간단하고 안정적인 CFI 구현 (execute_subscription만 사용)

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
#include <mutex>
#include <cxxabi.h>

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
static std::unordered_map<void*, size_t> g_subscription_hashes;  // subscription -> hash
static std::unordered_map<void*, int> g_violation_counts;  // subscription -> violation count

// 통계
static size_t g_total_calls = 0;
static size_t g_total_violations = 0;

// 원본 함수
static void (*original_execute_subscription)(void*, void*) = nullptr;

// 환경 변수
static bool g_cfi_enabled = true;
static bool g_cfi_strict = false;

// Subscription 객체의 해시 계산 (간단한 체크섬)
size_t calculate_hash(void* subscription) {
    unsigned char* bytes = (unsigned char*)subscription;
    size_t hash = 0;
    
    // 처음 256 바이트의 해시
    for (size_t i = 0; i < 256; i++) {
        hash = hash * 31 + bytes[i];
    }
    
    return hash;
}

// 생성자
__attribute__((constructor))
void cfi_init() {
    LOG("=== Simple Attack Detector CFI Loaded ===");
    LOG("PID: %d", getpid());
    
    // 환경 변수
    if (getenv("ROS2_CFI_DISABLE")) {
        g_cfi_enabled = false;
        WARNING("CFI is DISABLED");
    }
    
    if (getenv("ROS2_CFI_STRICT")) {
        g_cfi_strict = true;
        LOG("Strict mode ENABLED");
    }
    
    // execute_subscription 후킹
    const char* symbol = "_ZN6rclcpp8Executor20execute_subscriptionESt10shared_ptrINS_16SubscriptionBaseEE";
    original_execute_subscription = (void (*)(void*, void*))dlsym(RTLD_NEXT, symbol);
    
    if (original_execute_subscription) {
        SUCCESS("Hooked execute_subscription at %p", original_execute_subscription);
    } else {
        ERROR("Failed to hook execute_subscription");
    }
}

// execute_subscription 후킹
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
        
        // 현재 해시 계산
        size_t current_hash = calculate_hash(subscription);
        
        // 첫 호출인지 확인
        auto hash_it = g_subscription_hashes.find(subscription);
        if (hash_it == g_subscription_hashes.end()) {
            // 첫 호출 - 해시 저장
            g_subscription_hashes[subscription] = current_hash;
            LOG("Registered subscription %p (hash: %zx)", subscription, current_hash);
        } else {
            // 이후 호출 - 해시 비교
            size_t original_hash = hash_it->second;
            
            if (current_hash != original_hash) {
                g_total_violations++;
                g_violation_counts[subscription]++;
                
                ERROR("🚨 MEMORY CORRUPTION DETECTED! (Violation #%zu)", g_total_violations);
                ERROR("  Subscription: %p", subscription);
                ERROR("  Original hash: %zx", original_hash);
                ERROR("  Current hash: %zx", current_hash);
                ERROR("  This subscription has %d violations", g_violation_counts[subscription]);
                
                // 메모리 덤프 (처음 64 바이트)
                ERROR("  Memory dump (first 64 bytes):");
                unsigned char* bytes = (unsigned char*)subscription;
                for (int i = 0; i < 64; i++) {
                    if (i % 16 == 0) fprintf(stderr, "\n    ");
                    fprintf(stderr, "%02x ", bytes[i]);
                }
                fprintf(stderr, "\n");
                
                if (g_cfi_strict) {
                    ERROR("  ❌ BLOCKING EXECUTION (strict mode)");
                    allow_execution = false;
                } else {
                    WARNING("  ⚠️  ALLOWING EXECUTION (non-strict mode)");
                }
            }
        }
    }
    
    // 원본 호출 또는 차단
    if (allow_execution && original_execute_subscription) {
        original_execute_subscription(executor_ptr, subscription_shared_ptr);
    } else if (!allow_execution) {
        ERROR("Execution BLOCKED by CFI");
        ERROR("Terminating process for safety");
        abort();
    }
}

// 소멸자
__attribute__((destructor))
void cfi_fini() {
    LOG("\n=== CFI Summary ===");
    LOG("Total callback executions: %zu", g_total_calls);
    LOG("Total violations detected: %zu", g_total_violations);
    LOG("Monitored subscriptions: %zu", g_subscription_hashes.size());
    
    if (g_total_violations > 0) {
        ERROR("⚠️  CFI detected %zu corruptions/attacks!", g_total_violations);
        
        // 각 subscription별 위반 횟수
        for (const auto& [sub, count] : g_violation_counts) {
            ERROR("  Subscription %p: %d violations", sub, count);
        }
    } else if (g_total_calls > 0) {
        SUCCESS("✅ No violations detected in %zu executions", g_total_calls);
    }
}
