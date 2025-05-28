#!/bin/bash
# simple_test_cfi.sh - 간단한 CFI 테스트

set -e

echo "=== Simple CFI Test ==="

cd ~/ros2_cfi_project/cfi_lib

# 1. 빌드
echo "Building working CFI..."
g++ -std=c++17 -fPIC -shared -o libworking_cfi.so working_cfi.cpp -ldl

# 2. 환경 설정
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# 3. 정상 동작 테스트
echo -e "\n>>> Test 1: Normal subscriber"
echo "Starting subscriber..."
(
    cd ~/ros2_ws
    export LD_PRELOAD=~/ros2_cfi_project/cfi_lib/libworking_cfi.so
    timeout 10s ros2 run basic_communication subscriber
) &

SUB_PID=$!
sleep 3

# 메시지 보내기
echo "Sending messages..."
ros2 topic pub --once /topic std_msgs/msg/String "data: 'Test 1'" 2>/dev/null
sleep 1
ros2 topic pub --once /topic std_msgs/msg/String "data: 'Test 2'" 2>/dev/null

wait $SUB_PID 2>/dev/null || true

echo -e "\n>>> Test 2: Vulnerable subscriber (if available)"
if ros2 pkg executables | grep -q vulnerable_subscriber; then
    echo "Testing vulnerable subscriber..."
    (
        cd ~/ros2_ws
        export LD_PRELOAD=~/ros2_cfi_project/cfi_lib/libworking_cfi.so
        export ROS2_CFI_STRICT=1
        timeout 10s ros2 run basic_communication vulnerable_subscriber
    ) &
    
    VULN_PID=$!
    sleep 3
    
    # 정상 메시지
    ros2 topic pub --once /vulnerable_topic std_msgs/msg/String "data: 'Normal'" 2>/dev/null
    sleep 1
    
    # 긴 메시지 (오버플로우 시도)
    LONG_MSG=$(python3 -c "print('A' * 100)")
    ros2 topic pub --once /vulnerable_topic std_msgs/msg/String "data: '$LONG_MSG'" 2>/dev/null
    
    wait $VULN_PID 2>/dev/null || true
else
    echo "vulnerable_subscriber not found, skipping..."
fi

echo -e "\nTest complete!"
