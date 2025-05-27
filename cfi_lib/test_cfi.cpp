// test_cfi.cpp
// CFI 라이브러리 기본 동작 테스트 프로그램

#include <iostream>
#include <dlfcn.h>
#include <unistd.h>
#include <cstring>

// 테스트용 콜백 함수들
void legitimate_callback() {
    std::cout << "Legitimate callback executed" << std::endl;
}

void another_legitimate_callback() {
    std::cout << "Another legitimate callback executed" << std::endl;
}

void malicious_function() {
    std::cout << "⚠️  MALICIOUS FUNCTION EXECUTED! This should be blocked by CFI!" << std::endl;
}

// 함수 포인터 타입
typedef void (*callback_fn)();

// 콜백을 저장하고 호출하는 간단한 클래스
class CallbackManager {
private:
    callback_fn callback_;
    
public:
    CallbackManager() : callback_(nullptr) {}
    
    void register_callback(callback_fn cb) {
        std::cout << "Registering callback at address: " << (void*)cb << std::endl;
        callback_ = cb;
    }
    
    void trigger_callback() {
        if (callback_) {
            std::cout << "Triggering callback at address: " << (void*)callback_ << std::endl;
            callback_();
        } else {
            std::cout << "No callback registered" << std::endl;
        }
    }
    
    // 취약한 함수 - buffer overflow를 시뮬레이션
    void vulnerable_function(const char* input) {
        char buffer[16];
        
        std::cout << "Buffer address: " << (void*)buffer << std::endl;
        std::cout << "Callback pointer address: " << (void*)&callback_ << std::endl;
        
        // 의도적으로 안전하지 않은 복사 (테스트용)
        strcpy(buffer, input);  // 취약점!
        
        std::cout << "Buffer content: " << buffer << std::endl;
    }
};

int main() {
    std::cout << "=== ROS2 CFI Test Program ===" << std::endl;
    std::cout << "PID: " << getpid() << std::endl;
    
    // LD_PRELOAD 확인
    const char* ld_preload = getenv("LD_PRELOAD");
    if (ld_preload) {
        std::cout << "LD_PRELOAD is set: " << ld_preload << std::endl;
    } else {
        std::cout << "WARNING: LD_PRELOAD is not set!" << std::endl;
    }
    
    // CFI 테스트 함수 호출 시도
    void* handle = dlopen(nullptr, RTLD_LAZY);
    if (handle) {
        typedef void (*test_fn)();
        test_fn cfi_test = (test_fn)dlsym(handle, "cfi_test_function");
        if (cfi_test) {
            std::cout << "\nCalling CFI test function..." << std::endl;
            cfi_test();
        } else {
            std::cout << "CFI test function not found (CFI library might not be loaded)" << std::endl;
        }
        dlclose(handle);
    }
    
    // 콜백 매니저 테스트
    CallbackManager manager;
    
    std::cout << "\n=== Test 1: Normal callback registration ===" << std::endl;
    manager.register_callback(legitimate_callback);
    manager.trigger_callback();
    
    std::cout << "\n=== Test 2: Changing callback ===" << std::endl;
    manager.register_callback(another_legitimate_callback);
    manager.trigger_callback();
    
    std::cout << "\n=== Test 3: Attempting to use malicious callback ===" << std::endl;
    std::cout << "Malicious function address: " << (void*)malicious_function << std::endl;
    manager.register_callback(malicious_function);
    manager.trigger_callback();
    
    std::cout << "\n=== Test 4: Buffer overflow simulation ===" << std::endl;
    std::cout << "Testing with safe input..." << std::endl;
    manager.vulnerable_function("Safe");
    manager.trigger_callback();
    
    std::cout << "\nWARNING: Attempting buffer overflow..." << std::endl;
    std::cout << "In real CFI implementation, this should be detected and blocked" << std::endl;
    
    // 이 부분은 실제로는 위험하므로 주석 처리
    // manager.vulnerable_function("This is a very long string that will overflow the buffer!");
    
    std::cout << "\n=== All tests completed ===" << std::endl;
    std::cout << "Check /tmp/ros2_cfi.log for detailed logs" << std::endl;
    
    return 0;
}
