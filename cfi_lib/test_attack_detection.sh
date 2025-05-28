#!/bin/bash
# test_attack_detection.sh - CFI 공격 탐지 테스트

set -e

# 색상 정의
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${GREEN}=== CFI Attack Detection Test ===${NC}"

cd ~/ros2_cfi_project/cfi_lib

# 1. 빌드
echo -e "\n${YELLOW}Building attack detector CFI...${NC}"
g++ -std=c++17 -fPIC -shared -o libattack_detector_cfi.so attack_detector_cfi.cpp -ldl

# 2. 환경 설정
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# Test 1: 정상 동작
echo -e "\n${BLUE}>>> Test 1: Normal Operation${NC}"
echo "Running normal subscriber..."

rm -f /tmp/cfi_normal.log
(
    cd ~/ros2_ws
    export LD_PRELOAD=~/ros2_cfi_project/cfi_lib/libattack_detector_cfi.so
    timeout 8s ros2 run basic_communication subscriber 2>&1 | tee /tmp/cfi_normal.log
) &

NORMAL_PID=$!
sleep 3

# 메시지 보내기
for i in {1..3}; do
    echo "Sending message $i"
    ros2 topic pub --once /topic std_msgs/msg/String "data: 'Normal Message $i'" 2>/dev/null
    sleep 1
done

wait $NORMAL_PID 2>/dev/null || true

# 결과 확인
echo -e "\n${YELLOW}Test 1 Results:${NC}"
if grep -q "Registered callback" /tmp/cfi_normal.log; then
    echo -e "${GREEN}✓ Callbacks registered${NC}"
fi
if grep -q "No CFI violations" /tmp/cfi_normal.log; then
    echo -e "${GREEN}✓ No violations detected${NC}"
fi

# Test 2: Vulnerable Node Test
echo -e "\n${BLUE}>>> Test 2: Vulnerable Node Attack Test${NC}"

# vulnerable_subscriber가 있는지 확인
if ! ros2 pkg executables | grep -q vulnerable_subscriber; then
    echo -e "${RED}vulnerable_subscriber not found!${NC}"
    echo "Please build the vulnerable_subscriber first"
    exit 1
fi

echo "Running vulnerable subscriber (non-strict mode)..."
rm -f /tmp/cfi_vulnerable.log

(
    cd ~/ros2_ws
    export LD_PRELOAD=~/ros2_cfi_project/cfi_lib/libattack_detector_cfi.so
    export ROS2_CFI_STRICT=0
    timeout 15s ros2 run basic_communication vulnerable_subscriber 2>&1 | tee /tmp/cfi_vulnerable.log
) &

VULN_PID=$!
sleep 3

# 정상 메시지
echo -e "\n${YELLOW}Sending normal message...${NC}"
ros2 topic pub --once /vulnerable_topic std_msgs/msg/String "data: 'Hello World'" 2>/dev/null
sleep 2

# 공격 메시지 준비
echo -e "\n${YELLOW}Preparing attack...${NC}"
# vulnerable_subscriber의 출력에서 공격 페이로드 찾기
ATTACK_CMD=$(grep -oP 'ros2 topic pub.*HEX:.*' /tmp/cfi_vulnerable.log | head -1)

if [ -z "$ATTACK_CMD" ]; then
    echo -e "${YELLOW}Using simple buffer overflow attack...${NC}"
    # 간단한 버퍼 오버플로우 (32 A + 추가 데이터)
    OVERFLOW="AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA\x00\x00\x00\x00\x00\x00\x00\x00"
    ros2 topic pub --once /vulnerable_topic std_msgs/msg/String "data: '$OVERFLOW'" 2>/dev/null
else
    echo -e "${YELLOW}Using exploit payload from vulnerable_subscriber...${NC}"
    eval "$ATTACK_CMD" 2>/dev/null
fi

sleep 3

# 추가 메시지로 변조 확인
echo -e "\n${YELLOW}Sending post-attack message...${NC}"
ros2 topic pub --once /vulnerable_topic std_msgs/msg/String "data: 'After attack'" 2>/dev/null
sleep 2

kill $VULN_PID 2>/dev/null || true
wait $VULN_PID 2>/dev/null || true

# 결과 확인
echo -e "\n${YELLOW}Test 2 Results:${NC}"
if grep -q "CALLBACK HIJACKING DETECTED" /tmp/cfi_vulnerable.log; then
    echo -e "${GREEN}✓ CFI detected callback hijacking!${NC}"
    grep -A5 "CALLBACK HIJACKING" /tmp/cfi_vulnerable.log | head -10
elif grep -q "Subscription object modified" /tmp/cfi_vulnerable.log; then
    echo -e "${GREEN}✓ CFI detected memory modification${NC}"
    grep -A3 "object modified" /tmp/cfi_vulnerable.log
else
    echo -e "${RED}✗ CFI did not detect the attack${NC}"
fi

# Test 3: Strict Mode
echo -e "\n${BLUE}>>> Test 3: Strict Mode Test${NC}"
echo "Running vulnerable subscriber (strict mode)..."
rm -f /tmp/cfi_strict.log

(
    cd ~/ros2_ws
    export LD_PRELOAD=~/ros2_cfi_project/cfi_lib/libattack_detector_cfi.so
    export ROS2_CFI_STRICT=1
    timeout 10s ros2 run basic_communication vulnerable_subscriber 2>&1 | tee /tmp/cfi_strict.log
) &

STRICT_PID=$!
sleep 3

# 같은 공격 시도
echo -e "\n${YELLOW}Attempting attack in strict mode...${NC}"
ros2 topic pub --once /vulnerable_topic std_msgs/msg/String "data: 'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA'" 2>/dev/null
sleep 2

# 프로세스 상태 확인
if ps -p $STRICT_PID > /dev/null 2>&1; then
    echo "Process still running, sending more messages..."
    ros2 topic pub --once /vulnerable_topic std_msgs/msg/String "data: 'Test'" 2>/dev/null
    sleep 2
    kill $STRICT_PID 2>/dev/null || true
else
    echo -e "${GREEN}✓ Process terminated (likely by CFI)${NC}"
fi

wait $STRICT_PID 2>/dev/null || true

# 최종 요약
echo -e "\n${GREEN}=== Test Summary ===${NC}"
echo "1. Normal operation: /tmp/cfi_normal.log"
echo "2. Attack detection: /tmp/cfi_vulnerable.log"
echo "3. Strict mode: /tmp/cfi_strict.log"

echo -e "\n${YELLOW}Key metrics:${NC}"
for log in /tmp/cfi_normal.log /tmp/cfi_vulnerable.log /tmp/cfi_strict.log; do
    if [ -f "$log" ]; then
        echo -e "\n$(basename $log):"
        grep -E "violations|Blocked|Monitored" "$log" | tail -5 || echo "  No summary found"
    fi
done

echo -e "\n${GREEN}Test complete!${NC}"
