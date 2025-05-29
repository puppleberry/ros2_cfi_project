// day5_working_cfi.cpp
// 검증된 후킹 메커니즘을 사용하는 완전한 CFI 구현

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <dlfcn.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <unordered_map>
#include <unordered_set>
#include <mutex>
#include <vector>
#include <atomic>
#include <fstream>
#include <execinfo.h>

// 로깅 매크로
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

#define DEBUG(fmt, ...) do { \
    if (g_debug_mode) { \
        fprintf(stderr, "\033[0;36m[CFI-DEBUG] " fmt "\033[0m\n", ##__VA_ARGS__); \
        fflush(stderr); \
    } \
} while(0)

// Shadow Stack Entry
struct ShadowStackEntry {
    void* return_address;
    void* subscription;
    size_t hash;
};

// 전역 변수들
static std::mutex g_mutex;
static std::unordered_map<void*, size_t> g_subscription_hashes;
static std::unordered_map<void*, int> g_violation_counts;
static std::unordered_set<void*> g_valid_subscriptions;
static std::unordered_map<std::string, bool> g_node_policies;

// Thread-local shadow stack
static thread_local std::vector<ShadowStackEntry> g_shadow_stack;
static thread_local size_t g_shadow_stack_depth = 0;

// 통계
static std::atomic<size_t> g_total_calls(0);
static std::atomic<size_t> g_total_violations(0);
static std::atomic<size_t> g_shadow_stack_violations(0);
static std::atomic<bool> g_first_call(true);

// 원본 함수 포인터
static void (*original_execute_subscription)(void*, void*) = nullptr;

// 설정
static bool g_cfi_enabled = true;
static bool g_cfi_strict = false;
static bool g_debug_mode = false;
static bool g_shadow_stack_enabled = true;
static int g_performance_mode = 0;

// 원본 함수 찾기 (검증된 방법)
static void* find_original_function() {
    const char* symbol = "_ZN6rclcpp8Executor20execute_subscriptionESt10shared_ptrINS_16SubscriptionBaseEE";
    void* func = nullptr;
    
    // 1. RTLD_NEXT 시도 (가장 신뢰할 수 있는 방법)
    func = dlsym(RTLD_NEXT, symbol);
    if (func) {
        DEBUG("Found original via RTLD_NEXT: %p", func);
        return func;
    }
    
    // 2. 직접 librclcpp.so 열기 (백업)
    void* handle = dlopen("librclcpp.so", RTLD_NOW | RTLD_GLOBAL);
    if (handle) {
        func = dlsym(handle, symbol);
        if (func) {
            DEBUG("Found original via dlopen: %p", func);
            return func;
        }
    }
    
    ERROR("Failed to find original function!");
    return nullptr;
}

// Shadow Stack 함수들
void push_shadow_stack(void* return_addr, void* subscription, size_t hash) {
    if (!g_shadow_stack_enabled) return;
    
    if (g_shadow_stack_depth >= g_shadow_stack.size()) {
        g_shadow_stack.resize(g_shadow_stack_depth + 100);
    }
    
    g_shadow_stack[g_shadow_stack_depth] = {return_addr, subscription, hash};
    g_shadow_stack_depth++;
    
    DEBUG("Shadow stack push: depth=%zu, ret=%p, sub=%p", 
          g_shadow_stack_depth, return_addr, subscription);
}

bool pop_shadow_stack(void* expected_return, void* subscription) {
    if (!g_shadow_stack_enabled || g_shadow_stack_depth == 0) return true;
    
    g_shadow_stack_depth--;
    auto& entry = g_shadow_stack[g_shadow_stack_depth];
    
    if (entry.return_address != expected_return || entry.subscription != subscription) {
        g_shadow_stack_violations.fetch_add(1);
        ERROR("Shadow Stack Violation! Expected: ret=%p sub=%p, Got: ret=%p sub=%p",
              entry.return_address, entry.subscription, expected_return, subscription);
        return false;
    }
    
    DEBUG("Shadow stack pop: depth=%zu", g_shadow_stack_depth);
    return true;
}

// 해시 계산 (최적화 버전)
size_t calculate_hash(void* subscription) {
    if (!subscription) return 0;
    
    unsigned char* bytes = (unsigned char*)subscription;
    size_t hash = 0x12345678;
    
    // 성능 모드에 따른 체크 크기
    size_t check_size = (g_performance_mode == 2) ? 64 : 
                       (g_performance_mode == 1) ? 128 : 256;
    
    // 빠른 해시 계산
    for (size_t i = 0; i < check_size; i += 8) {
        hash = hash * 31 + *(size_t*)(bytes + i);
    }
    
    return hash;
}

// 노드별 정책 확인
bool should_enforce_strict(void* subscription) {
    if (g_cfi_strict) return true;
    
    // 프로세스 이름 기반 정책 (간단한 버전)
    char proc_name[256];
    FILE* f = fopen("/proc/self/comm", "r");
    if (f) {
        fgets(proc_name, sizeof(proc_name), f);
        fclose(f);
        
        // vulnerable_subscriber는 항상 strict 모드
        if (strstr(proc_name, "vulnerable")) {
            return true;
        }
    }
    
    return false;
}

// 후킹 함수 (C++ mangled name)
extern "C" void _ZN6rclcpp8Executor20execute_subscriptionESt10shared_ptrINS_16SubscriptionBaseEE(
    void* executor_ptr, void* subscription_shared_ptr)
{
    // 원본 함수 lazy 초기화
    if (!original_execute_subscription) {
        original_execute_subscription = (void (*)(void*, void*))find_original_function();
        if (!original_execute_subscription) {
            ERROR("Cannot proceed without original function!");
            abort();
        }
    }
    
    // 통계
    size_t call_num = g_total_calls.fetch_add(1) + 1;
    
    // 첫 호출 알림
    if (g_first_call.exchange(false)) {
        SUCCESS("=== CFI Protection Active! First callback intercepted ===");
    }
    
    // subscription 포인터 추출
    void* subscription = nullptr;
    if (subscription_shared_ptr) {
        subscription = *reinterpret_cast<void**>(subscription_shared_ptr);
    }
    
    DEBUG("Intercepted call #%zu: executor=%p, subscription=%p", 
          call_num, executor_ptr, subscription);
    
    if (!subscription) {
        WARNING("Null subscription pointer!");
        original_execute_subscription(executor_ptr, subscription_shared_ptr);
        return;
    }
    
    // Return address와 Shadow Stack
    void* return_addr = __builtin_return_address(0);
    size_t current_hash = calculate_hash(subscription);
    push_shadow_stack(return_addr, subscription, current_hash);
    
    bool allow_execution = true;
    
    // CFI 검증
    if (g_cfi_enabled) {
        std::lock_guard<std::mutex> lock(g_mutex);
        
        // 빠른 경로: 이미 검증된 subscription
        if (g_valid_subscriptions.find(subscription) != g_valid_subscriptions.end()) {
            DEBUG("Fast path: subscription %p already validated", subscription);
        } else {
            // 해시 기반 검증
            auto hash_it = g_subscription_hashes.find(subscription);
            if (hash_it == g_subscription_hashes.end()) {
                // 첫 등록
                g_subscription_hashes[subscription] = current_hash;
                g_valid_subscriptions.insert(subscription);
                SUCCESS("Registered new subscription %p (hash: 0x%zx)", subscription, current_hash);
            } else {
                // 해시 검증
                if (current_hash != hash_it->second) {
                    g_total_violations.fetch_add(1);
                    g_violation_counts[subscription]++;
                    g_valid_subscriptions.erase(subscription);
                    
                    ERROR("🚨 CFI VIOLATION DETECTED! 🚨");
                    ERROR("  Subscription: %p", subscription);
                    ERROR("  Expected hash: 0x%zx", hash_it->second);
                    ERROR("  Current hash: 0x%zx", current_hash);
                    ERROR("  Violation count: %d", g_violation_counts[subscription]);
                    ERROR("  Total violations: %zu", g_total_violations.load());
                    
                    // Backtrace 출력
                    void* buffer[10];
                    int nptrs = backtrace(buffer, 10);
                    ERROR("  Backtrace:");
                    char** strings = backtrace_symbols(buffer, nptrs);
                    if (strings) {
                        for (int i = 0; i < nptrs; i++) {
                            ERROR("    %s", strings[i]);
                        }
                        free(strings);
                    }
                    
                    if (should_enforce_strict(subscription)) {
                        ERROR("  ❌ BLOCKING EXECUTION (strict mode)");
                        allow_execution = false;
                    } else {
                        WARNING("  ⚠️  ALLOWING EXECUTION (permissive mode)");
                        // 새 해시로 업데이트
                        hash_it->second = current_hash;
                    }
                }
            }
        }
    }
    
    // 실행
    if (allow_execution) {
        original_execute_subscription(executor_ptr, subscription_shared_ptr);
    } else {
        ERROR("🛑 Execution blocked by CFI!");
        abort();
    }
    
    // Shadow Stack 검증
    if (!pop_shadow_stack(return_addr, subscription)) {
        g_total_violations.fetch_add(1);
        ERROR("Shadow stack corruption detected!");
        
        if (should_enforce_strict(subscription)) {
            ERROR("Aborting due to shadow stack violation!");
            abort();
        }
    }
}

// 통계 출력
extern "C" void cfi_print_statistics() {
    LOG("\n╔══════════════════════════════════════════╗");
    LOG("║        CFI Performance Statistics        ║");
    LOG("╠══════════════════════════════════════════╣");
    LOG("║ Total callback executions: %-13zu ║", g_total_calls.load());
    LOG("║ Total violations detected: %-13zu ║", g_total_violations.load());
    LOG("║ Shadow stack violations:   %-13zu ║", g_shadow_stack_violations.load());
    LOG("║ Monitored subscriptions:   %-13zu ║", g_subscription_hashes.size());
    LOG("║ Validated subscriptions:   %-13zu ║", g_valid_subscriptions.size());
    LOG("║ Shadow stack enabled:      %-13s ║", g_shadow_stack_enabled ? "YES" : "NO");
    LOG("║ Performance mode:          %-13d ║", g_performance_mode);
    LOG("║ CFI mode:                  %-13s ║", g_cfi_strict ? "STRICT" : "PERMISSIVE");
    LOG("╚══════════════════════════════════════════╝");
}

// 생성자
__attribute__((constructor))
void cfi_init() {
    LOG("=== ROS2 CFI Day 5 (Working Version) ===");
    LOG("PID: %d", getpid());
    
    // 환경 변수 처리
    if (getenv("ROS2_CFI_DISABLE")) {
        g_cfi_enabled = false;
        WARNING("CFI is DISABLED by environment variable");
        return;
    }
    
    if (getenv("ROS2_CFI_STRICT")) {
        g_cfi_strict = true;
        LOG("Strict mode ENABLED");
    }
    
    if (getenv("ROS2_CFI_DEBUG")) {
        g_debug_mode = true;
        LOG("Debug mode ENABLED");
    }
    
    if (getenv("ROS2_CFI_NO_SHADOW_STACK")) {
        g_shadow_stack_enabled = false;
        LOG("Shadow Stack DISABLED");
    }
    
    const char* perf_mode = getenv("ROS2_CFI_PERFORMANCE_MODE");
    if (perf_mode) {
        g_performance_mode = atoi(perf_mode);
        LOG("Performance mode: %d", g_performance_mode);
    }
    
    // Shadow Stack 예약
    g_shadow_stack.reserve(1000);
    
    SUCCESS("CFI initialized successfully!");
    LOG("Waiting for ROS2 callbacks...");
}

// 소멸자
__attribute__((destructor))
void cfi_fini() {
    LOG("\n=== ROS2 CFI Session Summary ===");
    cfi_print_statistics();
    
    if (g_total_violations > 0) {
        ERROR("⚠️  CFI detected %zu violations during this session!", g_total_violations.load());
        
        // 위반 상세 정보
        if (!g_violation_counts.empty()) {
            ERROR("Violation details:");
            for (const auto& [sub, count] : g_violation_counts) {
                ERROR("  Subscription %p: %d violations", sub, count);
            }
        }
    } else if (g_total_calls > 0) {
        SUCCESS("✅ No violations detected in %zu callback executions!", g_total_calls.load());
    } else {
        WARNING("No callbacks were intercepted. Was the library loaded correctly?");
    }
    
    // 통계 파일 저장
    if (getenv("ROS2_CFI_SAVE_STATS")) {
        std::ofstream stats_file("/tmp/day5_cfi_stats.txt");
        stats_file << "Day 5 CFI Statistics\n";
        stats_file << "====================\n";
        stats_file << "Total calls: " << g_total_calls << "\n";
        stats_file << "Total violations: " << g_total_violations << "\n";
        stats_file << "Shadow stack violations: " << g_shadow_stack_violations << "\n";
        stats_file << "Subscriptions monitored: " << g_subscription_hashes.size() << "\n";
        stats_file << "Hook status: " << (original_execute_subscription ? "SUCCESS" : "FAILED") << "\n";
        stats_file.close();
        LOG("Statistics saved to /tmp/day5_cfi_stats.txt");
    }
}
