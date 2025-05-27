// ros2_cfi_lib.cpp
// ROS2 CFI (Control Flow Integrity) Library
// LD_PRELOAD를 통해 ROS2 콜백 함수들을 보호하는 라이브러리

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <dlfcn.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <execinfo.h>
#include <cxxabi.h>
#include <cstdarg>      // va_start, va_end를 위해 추가
#include <unordered_set>
#include <unordered_map>
#include <mutex>
#include <fstream>
#include <iostream>
#include <sstream>
#include <chrono>

// 로깅 매크로
#define CFI_LOG_FILE "/tmp/ros2_cfi.log"
#define CFI_LOG(level, fmt, ...) cfi_log(level, __FILE__, __LINE__, fmt, ##__VA_ARGS__)

// 로그 레벨
enum LogLevel {
    LOG_DEBUG = 0,
    LOG_INFO,
    LOG_WARN,
    LOG_ERROR,
    LOG_FATAL
};

// 전역 변수들
static std::mutex g_mutex;
static std::unordered_set<void*> g_whitelist;  // 합법적인 콜백 함수 주소들
static std::unordered_map<void*, std::string> g_callback_info;  // 콜백 함수 정보
static FILE* g_log_file = nullptr;
static LogLevel g_log_level = LOG_INFO;
static bool g_initialized = false;

// 원본 함수 포인터들 (나중에 dlsym으로 채워질 예정)
typedef void* (*create_subscription_fn)(void*, const char*, size_t, void*, void*);
// TODO: 실제 후킹 구현 시 사용될 예정
// static create_subscription_fn original_create_subscription = nullptr;

// 로깅 함수
void cfi_log(LogLevel level, const char* file, int line, const char* fmt, ...) {
    if (level < g_log_level) return;
    
    static const char* level_str[] = {"DEBUG", "INFO", "WARN", "ERROR", "FATAL"};
    
    // 시간 정보 획득
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    struct tm* tm_info = localtime(&time_t);
    
    // 로그 메시지 작성
    char time_buf[32];
    strftime(time_buf, sizeof(time_buf), "%Y-%m-%d %H:%M:%S", tm_info);
    
    // 파일과 콘솔에 동시 출력
    FILE* outputs[] = {g_log_file, stderr};
    for (FILE* out : outputs) {
        if (out) {
            fprintf(out, "[%s] [%s] [%s:%d] ", time_buf, level_str[level], file, line);
            
            va_list args;
            va_start(args, fmt);
            vfprintf(out, fmt, args);
            va_end(args);
            
            fprintf(out, "\n");
            fflush(out);
        }
    }
    
    // FATAL 레벨인 경우 프로그램 종료
    if (level == LOG_FATAL) {
        abort();
    }
}

// 스택 트레이스 출력 함수
void print_backtrace() {
    void* buffer[100];
    int nptrs = backtrace(buffer, 100);
    char** strings = backtrace_symbols(buffer, nptrs);
    
    if (strings) {
        CFI_LOG(LOG_INFO, "=== Backtrace ===");
        for (int i = 0; i < nptrs; i++) {
            CFI_LOG(LOG_INFO, "  %s", strings[i]);
        }
        free(strings);
    }
}

// 함수 이름을 demangle하는 유틸리티
std::string demangle_name(const char* name) {
    int status = 0;
    char* demangled = abi::__cxa_demangle(name, nullptr, nullptr, &status);
    
    std::string result = (status == 0 && demangled) ? demangled : name;
    
    if (demangled) {
        free(demangled);
    }
    
    return result;
}

// 프로세스 정보 출력
void print_process_info() {
    CFI_LOG(LOG_INFO, "=== Process Information ===");
    CFI_LOG(LOG_INFO, "PID: %d", getpid());
    
    char exe_path[256];
    ssize_t len = readlink("/proc/self/exe", exe_path, sizeof(exe_path) - 1);
    if (len != -1) {
        exe_path[len] = '\0';
        CFI_LOG(LOG_INFO, "Executable: %s", exe_path);
    }
    
    // 환경 변수 확인
    const char* ld_preload = getenv("LD_PRELOAD");
    if (ld_preload) {
        CFI_LOG(LOG_INFO, "LD_PRELOAD: %s", ld_preload);
    }
}

// 라이브러리 초기화 함수 (프로그램 시작 시 자동 호출)
__attribute__((constructor))
void cfi_library_init() {
    // 이미 초기화되었으면 스킵
    if (g_initialized) return;
    
    // 로그 파일 열기
    g_log_file = fopen(CFI_LOG_FILE, "a");
    if (!g_log_file) {
        fprintf(stderr, "[CFI] Failed to open log file: %s\n", CFI_LOG_FILE);
        g_log_file = stderr;
    }
    
    CFI_LOG(LOG_INFO, "=== ROS2 CFI Library Initialized ===");
    
    // 환경 변수로 로그 레벨 설정
    const char* log_level_env = getenv("ROS2_CFI_LOG_LEVEL");
    if (log_level_env) {
        if (strcmp(log_level_env, "DEBUG") == 0) g_log_level = LOG_DEBUG;
        else if (strcmp(log_level_env, "INFO") == 0) g_log_level = LOG_INFO;
        else if (strcmp(log_level_env, "WARN") == 0) g_log_level = LOG_WARN;
        else if (strcmp(log_level_env, "ERROR") == 0) g_log_level = LOG_ERROR;
        
        CFI_LOG(LOG_INFO, "Log level set to: %s", log_level_env);
    }
    
    // 프로세스 정보 출력
    print_process_info();
    
    // TODO: 여기서 원본 ROS2 함수들의 주소를 dlsym으로 획득할 예정
    CFI_LOG(LOG_INFO, "Searching for ROS2 functions to hook...");
    
    // 예시: create_subscription 함수 찾기 (실제 함수명은 나중에 확인 필요)
    // original_create_subscription = (create_subscription_fn)dlsym(RTLD_NEXT, "_ZN6rclcpp4Node19create_subscription...");
    
    g_initialized = true;
    CFI_LOG(LOG_INFO, "CFI Library initialization complete");
}

// 라이브러리 종료 함수 (프로그램 종료 시 자동 호출)
__attribute__((destructor))
void cfi_library_fini() {
    CFI_LOG(LOG_INFO, "=== ROS2 CFI Library Shutting Down ===");
    
    // 통계 출력
    {
        std::lock_guard<std::mutex> lock(g_mutex);
        CFI_LOG(LOG_INFO, "Total callbacks registered: %zu", g_whitelist.size());
        
        if (g_log_level <= LOG_DEBUG) {
            CFI_LOG(LOG_DEBUG, "Registered callback addresses:");
            for (const auto& addr : g_whitelist) {
                auto it = g_callback_info.find(addr);
                if (it != g_callback_info.end()) {
                    CFI_LOG(LOG_DEBUG, "  %p: %s", addr, it->second.c_str());
                } else {
                    CFI_LOG(LOG_DEBUG, "  %p: (unknown)", addr);
                }
            }
        }
    }
    
    CFI_LOG(LOG_INFO, "CFI protection ended");
    
    if (g_log_file && g_log_file != stderr) {
        fclose(g_log_file);
    }
}

// 화이트리스트에 콜백 추가
void cfi_add_callback(void* callback_addr, const std::string& info) {
    std::lock_guard<std::mutex> lock(g_mutex);
    
    if (g_whitelist.insert(callback_addr).second) {
        g_callback_info[callback_addr] = info;
        CFI_LOG(LOG_INFO, "Callback registered: %p (%s)", callback_addr, info.c_str());
    } else {
        CFI_LOG(LOG_DEBUG, "Callback already registered: %p", callback_addr);
    }
}

// 콜백 주소 검증
bool cfi_verify_callback(void* callback_addr) {
    std::lock_guard<std::mutex> lock(g_mutex);
    
    bool is_valid = g_whitelist.find(callback_addr) != g_whitelist.end();
    
    if (!is_valid) {
        CFI_LOG(LOG_ERROR, "INVALID callback detected: %p", callback_addr);
        CFI_LOG(LOG_ERROR, "This could be a control flow hijacking attempt!");
        print_backtrace();
    } else {
        CFI_LOG(LOG_DEBUG, "Callback verified: %p", callback_addr);
    }
    
    return is_valid;
}

// CFI 정책 적용 (비정상 콜백 감지 시 동작)
void cfi_policy_violation(void* invalid_callback) {
    CFI_LOG(LOG_FATAL, "CFI POLICY VIOLATION! Invalid callback: %p", invalid_callback);
    CFI_LOG(LOG_FATAL, "Terminating process to prevent potential exploitation");
    
    // 스택 덤프
    print_backtrace();
    
    // 메모리 맵 출력 (디버깅용)
    if (g_log_level <= LOG_DEBUG) {
        CFI_LOG(LOG_DEBUG, "=== Memory Maps ===");
        std::ifstream maps("/proc/self/maps");
        std::string line;
        while (std::getline(maps, line)) {
            CFI_LOG(LOG_DEBUG, "%s", line.c_str());
        }
    }
    
    // 프로그램 종료
    abort();
}

// 테스트용 더미 함수 (나중에 실제 후킹 함수로 대체)
extern "C" void cfi_test_function() {
    CFI_LOG(LOG_INFO, "CFI test function called");
    
    // 테스트: 가짜 콜백 주소로 검증 시도
    void* fake_callback = (void*)0xdeadbeef;
    if (!cfi_verify_callback(fake_callback)) {
        CFI_LOG(LOG_WARN, "Test: Invalid callback correctly detected");
    }
}

// 환경 변수로 CFI 활성화/비활성화 제어
bool is_cfi_enabled() {
    static bool checked = false;
    static bool enabled = true;
    
    if (!checked) {
        const char* env = getenv("ROS2_CFI_DISABLE");
        enabled = (env == nullptr || strcmp(env, "1") != 0);
        checked = true;
        
        CFI_LOG(LOG_INFO, "CFI protection is %s", enabled ? "ENABLED" : "DISABLED");
    }
    
    return enabled;
}
