#!/bin/bash
# test_final_cfi.sh - 최종 CFI 테스트

set -e

# 색상 정의
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${GREEN}=== ROS2 CFI Final Test ===${NC}"

cd ~/ros2_cfi_project/cfi_lib

# 1. CFI 라이브러리 빌드
echo -e "\n${YELLOW}Building final CFI library...${NC}"
g++ -std=c++17 -fPIC -shared -o libros2_cfi_final.so final_callback_hook.cpp -ldl

# 2. ROS2 환경 설정
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# 테스트 함수
run_test() {
    local test_name=$1
    local command=$2
    local env_vars=$3
    local duration=$4
    
    echo -e "\n${BLUE}=== Test: $test_name ===${NC}"
    echo "Environment: $env_vars"
    
    rm -f /tmp/cfi_test_${test_name}.log
    
    # 노드 실행 (백그라운드)
    (
        cd ~/ros2_ws
        export LD_PRELOAD=~/ros2_cfi_project/cfi_lib/libros2_cfi_final.so
        eval "export $env_vars"
        timeout ${duration}s $command 2>&1 | tee /tmp/cfi_test_${test_name}.log
    ) &
    
    NODE_PID=$!
    
    return $NODE_PID
}

# Test 1: 정상 동작 확인
echo -e "\n${GREEN}>>> Test 1: Normal Operation${NC}"
run_test "normal" "ros2 run basic_communication subscriber" "ROS2_CFI_STRICT=0" 10
NORMAL_PID=$!

sleep 3

# 정상 메시지 발송
for i in {1..3}; do
    echo "Sending normal message $i"
    ros2 topic pub --once /topic std_msgs/msg/String "data: 'Normal Message $i'" 2>/dev/null
    sleep 1
done

wait $NORMAL_PID 2>/dev/null || true

# 결과 확인
echo -e "\n${YELLOW}Test 1 Results:${NC}"
if grep -q "Registered callback" /tmp/cfi_test_normal.log; then
    echo -e "${GREEN}✓ Callbacks registered${NC}"
    grep "Registered callback" /tmp/cfi_test_normal.log | head -3
fi
if grep -q "No CFI violations" /tmp/cfi_test_normal.log; then
    echo -e "${GREEN}✓ No violations detected${NC}"
fi

# Test 2: 취약한 노드 - Non-strict 모드
echo -e "\n${GREEN}>>> Test 2: Vulnerable Node (Non-strict Mode)${NC}"
run_test "vulnerable_nonstrict" "ros2 run basic_communication vulnerable_subscriber" "ROS2_CFI_STRICT=0" 15
VULN_PID=$!

sleep 3

# 정상 메시지
echo -e "\n${YELLOW}Sending normal message to vulnerable node...${NC}"
ros2 topic pub --once /vulnerable_topic std_msgs/msg/String "data: 'Hello World'" 2>/dev/null
sleep 2

# 공격 메시지 (버퍼 오버플로우)
echo -e "\n${YELLOW}Sending attack payload...${NC}"
# 32바이트 'A' + 악성 함수 포인터를 시뮬레이션
ATTACK_PAYLOAD="AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"
ros2 topic pub --once /vulnerable_topic std_msgs/msg/String "data: '$ATTACK_PAYLOAD'" 2>/dev/null
sleep 2

kill $VULN_PID 2>/dev/null || true
wait $VULN_PID 2>/dev/null || true

# 결과 확인
echo -e "\n${YELLOW}Test 2 Results:${NC}"
if grep -q "CFI VIOLATION DETECTED" /tmp/cfi_test_vulnerable_nonstrict.log; then
    echo -e "${GREEN}✓ CFI detected the attack${NC}"
    grep -A5 "CFI VIOLATION" /tmp/cfi_test_vulnerable_nonstrict.log | head -10
else
    echo -e "${RED}✗ CFI did not detect the attack${NC}"
fi

# Test 3: 취약한 노드 - Strict 모드
echo -e "\n${GREEN}>>> Test 3: Vulnerable Node (Strict Mode)${NC}"
echo -e "${YELLOW}This test may crash the node if an attack is detected${NC}"

run_test "vulnerable_strict" "ros2 run basic_communication vulnerable_subscriber" "ROS2_CFI_STRICT=1" 15
STRICT_PID=$!

sleep 3

# 정상 메시지
echo -e "\n${YELLOW}Sending normal message...${NC}"
ros2 topic pub --once /vulnerable_topic std_msgs/msg/String "data: 'Safe Message'" 2>/dev/null
sleep 2

# 공격 메시지
echo -e "\n${YELLOW}Sending attack payload (strict mode)...${NC}"
ros2 topic pub --once /vulnerable_topic std_msgs/msg/String "data: '$ATTACK_PAYLOAD'" 2>/dev/null
sleep 2

# 프로세스가 여전히 실행 중인지 확인
if ps -p $STRICT_PID > /dev/null 2>&1; then
    echo -e "${YELLOW}Process still running, terminating...${NC}"
    kill $STRICT_PID 2>/dev/null || true
else
    echo -e "${GREEN}✓ Process terminated (likely due to CFI)${NC}"
fi

wait $STRICT_PID 2>/dev/null || true

# 결과 확인
echo -e "\n${YELLOW}Test 3 Results:${NC}"
if grep -q "BLOCKING EXECUTION" /tmp/cfi_test_vulnerable_strict.log; then
    echo -e "${GREEN}✓ CFI blocked the malicious callback${NC}"
fi
if grep -q "FATAL: CFI violation" /tmp/cfi_test_vulnerable_strict.log; then
    echo -e "${GREEN}✓ Strict mode terminated the process${NC}"
fi

# 최종 요약
echo -e "\n${GREEN}=== Final Summary ===${NC}"
echo "Test logs created:"
ls -la /tmp/cfi_test_*.log 2>/dev/null

echo -e "\n${YELLOW}Key findings:${NC}"
echo "1. Normal operation log: /tmp/cfi_test_normal.log"
echo "2. Attack detection (non-strict): /tmp/cfi_test_vulnerable_nonstrict.log"
echo "3. Attack prevention (strict): /tmp/cfi_test_vulnerable_strict.log"

echo -e "\n${GREEN}CFI Protection Test Complete!${NC}"
