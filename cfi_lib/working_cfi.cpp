// working_cfi.cpp
// 실제로 작동하는 간단한 CFI 구현

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

#define LOG(fmt, ...) do { \
    fprintf(stderr, "[CFI] " fmt "\n", ##__VA_ARGS__); \
    fflush(stderr); \
} while(0)

// 전역 변수
static std::mutex g_mutex;
static std::unordered_map<void*, bool> g_subscription_state;  // subscription -> is_first_call
static size_t g_total_calls = 0;
static size_t g_violations = 0;

// 원본 함수
static void (*original_execute_subscription)(void*, void*) = nullptr;

// 환경 변수
static bool g_cfi_enabled = true;
static bool g_cfi_strict = false;

// 간단한 콜백 변조 감지
bool detect_callback_tampering(void* subscription) {
    // 매우 간단한 휴리스틱: subscription 객체의 특정 영역이 변경되었는지 확인
    static std::unordered_map<void*, size_t> g_checksums;
    
    // 간단한 체크섬 계산 (처음 128 바이트)
    unsigned char* bytes = (unsigned char*)subscription;
    size_t checksum = 0;
    for (int i = 0; i < 128; i++) {
        checksum = checksum * 31 + bytes[i];
    }
    
    std::lock_guard<std::mutex> lock(g_mutex);
    
    auto it = g_checksums.find(subscription);
    if (it == g_checksums.end()) {
        // 첫 호출
        g_checksums[subscription] = checksum;
        return false;
    } else {
        // 체크섬 비교
        if (it->second != checksum) {
            LOG("WARNING: Subscription object modified! (checksum mismatch)");
            return true;
        }
        return false;
    }
}

// 생성자
__attribute__((constructor))
void cfi_init() {
    LOG("=== Working CFI Loaded ===");
    LOG("PID: %d", getpid());
    
    // 환경 변수
    if (getenv("ROS2_CFI_DISABLE")) {
        g_cfi_enabled = false;
        LOG("CFI DISABLED");
    }
    if (getenv("ROS2_CFI_STRICT")) {
        g_cfi_strict = true;
        LOG("Strict mode ENABLED");
    }
    
    // 후킹
    const char* symbol = "_ZN6rclcpp8Executor20execute_subscriptionESt10shared_ptrINS_16SubscriptionBaseEE";
    original_execute_subscription = (void (*)(void*, void*))dlsym(RTLD_NEXT, symbol);
    
    if (original_execute_subscription) {
        LOG("Hooked execute_subscription successfully");
    } else {
        LOG("Failed to hook execute_subscription");
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
    
    bool allow_execution = true;
    
    if (g_cfi_enabled && subscription) {
        // 간단한 변조 감지
        if (detect_callback_tampering(subscription)) {
            g_violations++;
            LOG("🚨 CFI VIOLATION #%zu DETECTED!", g_violations);
            LOG("  Subscription: %p", subscription);
            LOG("  Call #%zu", g_total_calls);
            
            if (g_cfi_strict) {
                LOG("  ❌ BLOCKING EXECUTION");
                allow_execution = false;
            } else {
                LOG("  ⚠️  ALLOWING EXECUTION (non-strict)");
            }
        }
    }
    
    // 원본 호출
    if (allow_execution && original_execute_subscription) {
        original_execute_subscription(executor_ptr, subscription_shared_ptr);
    } else if (!allow_execution) {
        LOG("Execution BLOCKED by CFI");
        abort();
    }
}

// 소멸자
__attribute__((destructor))
void cfi_fini() {
    LOG("\n=== CFI Summary ===");
    LOG("Total calls: %zu", g_total_calls);
    LOG("Violations detected: %zu", g_violations);
    
    if (g_violations > 0) {
        LOG("⚠️  CFI detected %zu potential attacks!", g_violations);
    } else if (g_total_calls > 0) {
        LOG("✅ No violations in %zu calls", g_total_calls);
    }
}
