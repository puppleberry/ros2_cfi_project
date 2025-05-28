// improved_cfi.cpp
// 개선된 CFI - 실제 콜백 함수 변경을 감지

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
#include <signal.h>
#include <setjmp.h>

#define LOG(fmt, ...) do { \
    fprintf(stderr, "[CFI] " fmt "\n", ##__VA_ARGS__); \
    fflush(stderr); \
} while(0)

#define SUCCESS(fmt, ...) do { \
    fprintf(stderr, "\033[0;32m[CFI] " fmt "\033[0m\n", ##__VA_ARGS__); \
    fflush(stderr); \
} while(0)

#define ERROR(fmt, ...) do { \
    fprintf(stderr, "\033[0;31m[CFI-ALERT] " fmt "\033[0m\n", ##__VA_ARGS__); \
    fflush(stderr); \
} while(0)

// 전역 변수
static std::mutex g_mutex;
static std::unordered_map<void*, void*> g_legitimate_callbacks;  // subscription -> legitimate callback
static std::unordered_set<void*> g_whitelist;  // 모든 합법적인 콜백 주소
static size_t g_total_calls = 0;
static size_t g_violations = 0;
static bool g_cfi_enabled = true;
static bool g_cfi_strict = false;

// 원본 함수
static void (*original_execute_subscription)(void*, void*) = nullptr;

// SIGSEGV 핸들러를 위한 jmp_buf
static sigjmp_buf g_jmpbuf;
static bool g_in_callback_check = false;

// 시그널 핸들러
void sigsegv_handler(int sig) {
    if (g_in_callback_check) {
        ERROR("Segmentation fault detected during callback check!");
        siglongjmp(g_jmpbuf, 1);
    }
    // 원래 핸들러로 전달
    signal(SIGSEGV, SIG_DFL);
    raise(sig);
}

// 메모리 읽기 가능한지 안전하게 확인
bool is_readable_memory(void* addr, size_t size) {
    // 간단한 방법: /proc/self/maps 확인
    FILE* fp = fopen("/proc/self/maps", "r");
    if (!fp) return false;
    
    char line[256];
    uintptr_t target = (uintptr_t)addr;
    
    while (fgets(line, sizeof(line), fp)) {
        uintptr_t start, end;
        char perms[5];
        if (sscanf(line, "%lx-%lx %4s", &start, &end, perms) == 3) {
            if (target >= start && target + size <= end && perms[0] == 'r') {
                fclose(fp);
                return true;
            }
        }
    }
    
    fclose(fp);
    return false;
}

// Subscription에서 현재 콜백 추출 시도
void* extract_current_callback(void* subscription) {
    if (!subscription) return nullptr;
    
    // 안전하게 메모리 접근
    g_in_callback_check = true;
    if (sigsetjmp(g_jmpbuf, 1) != 0) {
        g_in_callback_check = false;
        return nullptr;
    }
    
    // 일반적인 콜백 오프셋들 (실험적)
    const size_t offsets[] = {48, 56, 64, 72, 80, 88, 96, 104, 112, 120, 128, 136, 144, 152, 160, 168};
    
    unsigned char* bytes = (unsigned char*)subscription;
    void* found_callback = nullptr;
    
    for (size_t offset : offsets) {
        if (!is_readable_memory(bytes + offset, sizeof(void*))) continue;
        
        void* potential = *(void**)(bytes + offset);
        if (!potential) continue;
        
        // 실행 가능한 메모리인지 확인
        if (is_readable_memory(potential, 1)) {
            // 함수 포인터처럼 보이는지 확인
            unsigned char* code = (unsigned char*)potential;
            // x86-64 함수 프롤로그 패턴 확인 (push rbp; mov rbp,rsp 등)
            if ((code[0] == 0x55 || code[0] == 0x48 || code[0] == 0xf3) && 
                is_readable_memory(code, 16)) {
                found_callback = potential;
                break;
            }
        }
    }
    
    g_in_callback_check = false;
    return found_callback;
}

// 생성자
__attribute__((constructor))
void cfi_init() {
    LOG("=== Improved CFI Protection Loaded ===");
    LOG("PID: %d", getpid());
    
    // 시그널 핸들러 설치
    signal(SIGSEGV, sigsegv_handler);
    
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
        SUCCESS("Hooked execute_subscription");
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
    
    bool allow_execution = true;
    
    if (g_cfi_enabled && subscription) {
        // 현재 콜백 추출
        void* current_callback = extract_current_callback(subscription);
        
        std::lock_guard<std::mutex> lock(g_mutex);
        
        // 첫 호출인지 확인
        auto it = g_legitimate_callbacks.find(subscription);
        if (it == g_legitimate_callbacks.end()) {
            // 첫 호출 - 콜백 등록
            if (current_callback) {
                g_legitimate_callbacks[subscription] = current_callback;
                g_whitelist.insert(current_callback);
                SUCCESS("Registered callback %p for subscription %p", current_callback, subscription);
            }
        } else {
            // 이후 호출 - 콜백 검증
            void* legitimate_callback = it->second;
            
            if (current_callback && current_callback != legitimate_callback) {
                g_violations++;
                ERROR("🚨 CFI VIOLATION DETECTED! (#%zu)", g_violations);
                ERROR("  Subscription: %p", subscription);
                ERROR("  Expected callback: %p", legitimate_callback);
                ERROR("  Current callback: %p", current_callback);
                
                // 화이트리스트에 있는지 확인
                if (g_whitelist.find(current_callback) == g_whitelist.end()) {
                    ERROR("  ⚠️  Current callback is NOT in whitelist!");
                    
                    if (g_cfi_strict) {
                        ERROR("  ❌ BLOCKING EXECUTION (strict mode)");
                        allow_execution = false;
                    }
                }
            }
        }
    }
    
    // 실행 전 한 번 더 체크
    if (allow_execution) {
        // 콜백 실행 중 크래시 감지를 위한 래퍼
        struct sigaction old_sa;
        struct sigaction sa;
        sa.sa_handler = [](int sig) {
            ERROR("🚨 CRASH DETECTED during callback execution!");
            ERROR("This might be due to corrupted callback pointer");
            abort();
        };
        sigemptyset(&sa.sa_mask);
        sa.sa_flags = 0;
        
        sigaction(SIGSEGV, &sa, &old_sa);
        
        if (original_execute_subscription) {
            original_execute_subscription(executor_ptr, subscription_shared_ptr);
        }
        
        sigaction(SIGSEGV, &old_sa, nullptr);
    } else {
        ERROR("Execution BLOCKED by CFI");
        if (g_cfi_strict) {
            abort();
        }
    }
}

// 소멸자
__attribute__((destructor))
void cfi_fini() {
    LOG("\n=== CFI Summary ===");
    LOG("Total executions: %zu", g_total_calls);
    LOG("Violations detected: %zu", g_violations);
    LOG("Registered callbacks: %zu", g_whitelist.size());
    
    if (g_violations > 0) {
        ERROR("⚠️  CFI detected %zu violations!", g_violations);
    } else if (g_total_calls > 0) {
        SUCCESS("✅ No violations in %zu executions", g_total_calls);
    }
}
