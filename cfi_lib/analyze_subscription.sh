#!/bin/bash
# analyze_subscription.sh - ROS2 Subscription 클래스 구조 분석

echo "=== Analyzing ROS2 Subscription Structure ==="

# 1. SubscriptionBase 관련 함수들 찾기
echo -e "\n[1] SubscriptionBase virtual functions:"
nm -D /opt/ros/humble/lib/librclcpp.so | grep -i "subscriptionbase" | grep -E "handle|execute|callback" | head -20

# 2. handle_message 관련 함수들
echo -e "\n[2] handle_message functions:"
nm -D /opt/ros/humble/lib/librclcpp.so | grep -i "handle_message" | head -10

# 3. execute 관련 함수들
echo -e "\n[3] execute functions:"
nm -D /opt/ros/humble/lib/librclcpp.so | grep -i "subscription.*execute" | head -10

# 4. AnySubscriptionCallback 관련 (실제 콜백이 저장되는 곳)
echo -e "\n[4] AnySubscriptionCallback:"
nm -D /opt/ros/humble/lib/librclcpp.so | grep -i "anysubscriptioncallback" | head -10

# 5. 실제 메시지 디스패치 함수들
echo -e "\n[5] Message dispatch functions:"
nm -D /opt/ros/humble/lib/librclcpp.so | grep -E "dispatch|invoke.*callback" | head -10

# 6. std_msgs String 관련 subscription
echo -e "\n[6] String message subscriptions:"
nm -D /opt/ros/humble/lib/librclcpp.so | grep "SubscriptionI.*String" | head -10

# 7. 가장 유망한 후킹 타겟 찾기
echo -e "\n[7] Most promising hooking targets:"
echo "Looking for functions that are called when messages arrive..."
nm -D /opt/ros/humble/lib/librclcpp.so | grep -E "_ZN.*Subscription.*handle_message|_ZN.*Subscription.*execute_impl|_ZN.*AnySubscriptionCallback.*dispatch" | head -20

# 8. 심볼 demangle
echo -e "\n[8] Demangling some key symbols:"
echo "_ZN6rclcpp16SubscriptionBase13handle_messageESt10shared_ptrIN3rcl3msg14MessageInfoPtrEEOS2_IvE" | c++filt
echo "_ZN6rclcpp16SubscriptionBase12execute_implESt10shared_ptrIvE" | c++filt

# 9. RTTI 정보 확인 (타입 정보)
echo -e "\n[9] RTTI Information:"
nm -D /opt/ros/humble/lib/librclcpp.so | grep -i "typeinfo.*subscription" | head -10
