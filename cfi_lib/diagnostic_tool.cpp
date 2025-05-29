// diagnostic_tool.cpp
#include <dlfcn.h>
#include <link.h>
#include <stdio.h>
#include <string.h>

// 로드된 모든 라이브러리와 execute_subscription 심볼 찾기
static int callback(struct dl_phdr_info *info, size_t size, void *data) {
    printf("Library: %s\n", info->dlpi_name);
    
    void* handle = dlopen(info->dlpi_name, RTLD_LAZY | RTLD_NOLOAD);
    if (handle) {
        void* sym = dlsym(handle, "_ZN6rclcpp8Executor20execute_subscriptionESt10shared_ptrINS_16SubscriptionBaseEE");
        if (sym) {
            printf("  -> Found execute_subscription at %p\n", sym);
        }
        dlclose(handle);
    }
    return 0;
}

__attribute__((constructor))
void diagnose() {
    printf("\n=== ROS2 Symbol Diagnostic ===\n");
    dl_iterate_phdr(callback, NULL);
    
    // 현재 실행 중인 execute_subscription 찾기
    void* current = dlsym(RTLD_DEFAULT, "_ZN6rclcpp8Executor20execute_subscriptionESt10shared_ptrINS_16SubscriptionBaseEE");
    printf("\nCurrent execute_subscription: %p\n", current);
}
