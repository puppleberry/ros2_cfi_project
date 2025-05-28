#!/bin/bash
# auto_test_cfi.sh - 자동화된 CFI 테스트

set -e

# 색상
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${GREEN}=== Automated CFI Test ===${NC}"

cd ~/ros2_cfi_project/cfi_lib

# 1. 빌드
echo "Building CFI..."
g++ -std=c++17 -fPIC -shared -o libsimple_detector.so simple_attack_detector.cpp -ldl

# 2. 환경 설정
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Test 1: 정상 동작
echo -e "\n${YELLOW}Test 1: Normal Operation${NC}"
(
    cd ~/ros2_ws
    export LD_PRELOAD=~/ros2_cfi_project/cfi_lib/libsimple_detector.so
    timeout 10s ros2 run basic_communication subscriber 2>&1
) &
SUB_PID=$!

sleep 3

# 메시지 보내기
for i in {1..3}; do
    ros2 topic pub --once /topic std_msgs/msg/String "data: 'Test $i'" 2>/dev/null
    sleep 1
done

wait $SUB_PID 2>/dev/null || true

# Test 2: Vulnerable Node (있는 경우)
if ros2 pkg executables | grep -q vulnerable_subscriber; then
    echo -e "\n${YELLOW}Test 2: Vulnerable Node${NC}"
    
    (
        cd ~/ros2_ws
        export LD_PRELOAD=~/ros2_cfi_project/cfi_lib/libsimple_detector.so
        export ROS2_CFI_STRICT=0
        timeout 15s ros2 run basic_communication vulnerable_subscriber 2>&1
    ) &
    VULN_PID=$!
    
    sleep 3
    
    # 정상 메시지
    echo "Sending normal message..."
    ros2 topic pub --once /vulnerable_topic std_msgs/msg/String "data: 'Normal'" 2>/dev/null
    sleep 2
    
    # 공격 메시지
    echo "Sending attack message..."
    # 40개의 A (버퍼 오버플로우)
    ATTACK="AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"
    ros2 topic pub --once /vulnerable_topic std_msgs/msg/String "data: '$ATTACK'" 2>/dev/null
    sleep 2
    
    # 공격 후 메시지
    echo "Sending post-attack message..."
    ros2 topic pub --once /vulnerable_topic std_msgs/msg/String "data: 'After attack'" 2>/dev/null
    sleep 2
    
    kill $VULN_PID 2>/dev/null || true
    wait $VULN_PID 2>/dev/null || true
else
    echo -e "${RED}vulnerable_subscriber not found${NC}"
fi

echo -e "\n${GREEN}Test Complete!${NC}"
