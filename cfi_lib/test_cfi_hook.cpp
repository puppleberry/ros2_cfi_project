#include <dlfcn.h>
#include <stdio.h>
#include <stdlib.h>

int main() {
    printf("=== CFI Hook Test ===\n");
    
    // 원본 심볼 찾기
    const char* symbol = "_ZN6rclcpp8Executor20execute_subscriptionESt10shared_ptrINS_16SubscriptionBaseEE";
    
    // RTLD_DEFAULT로 찾기
    void* func1 = dlsym(RTLD_DEFAULT, symbol);
    printf("RTLD_DEFAULT: %p\n", func1);
    
    // RTLD_NEXT로 찾기
    void* func2 = dlsym(RTLD_NEXT, symbol);
    printf("RTLD_NEXT: %p\n", func2);
    
    // librclcpp.so 직접 열기
    void* handle = dlopen("librclcpp.so", RTLD_NOW | RTLD_GLOBAL);
    if (handle) {
        void* func3 = dlsym(handle, symbol);
        printf("Direct dlopen: %p\n", func3);
        dlclose(handle);
    } else {
        printf("Failed to open librclcpp.so: %s\n", dlerror());
    }
    
    return 0;
}
