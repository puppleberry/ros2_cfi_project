#!/bin/bash
# test_callback_detector.sh - 콜백 감지 테스트 (타이밍 문제 해결)

set -e

# 색상 정의
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${GREEN}=== Callback Detection Test ===${NC}"

cd ~/ros2_cfi_project/cfi_lib

# 1. 빌드
echo -e "\n${YELLOW}Building callback detector...${NC}"
g++ -std=c++17 -fPIC -shared -o libcallback_detector.so simple_callback_detector.cpp -ldl

# 2. ROS2 환경 설정
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# 3. 로그 초기화
rm -f /tmp/callback_detector.log

# 4. Publisher를 먼저 시작 (백그라운드)
echo -e "\n${YELLOW}Starting publisher in background...${NC}"
(
    sleep 3  # subscriber가 준비될 때까지 대기
    echo -e "${GREEN}Sending messages...${NC}"
    for i in {1..5}; do
        echo "Publishing message $i"
        ros2 topic pub --once /topic std_msgs/msg/String "data: 'Test Message $i'" 2>/dev/null
        sleep 1
    done
) &

PUB_PID=$!

# 5. Subscriber 실행 (콜백 감지 활성화)
echo -e "\n${YELLOW}Starting subscriber with callback detection...${NC}"
cd ~/ros2_ws
LD_PRELOAD=~/ros2_cfi_project/cfi_lib/libcallback_detector.so \
timeout 10s ros2 run basic_communication subscriber 2>&1 | tee /tmp/callback_detector.log

# 6. Publisher 종료 대기
wait $PUB_PID 2>/dev/null || true

# 7. 결과 분석
echo -e "\n${GREEN}=== Results ===${NC}"

if grep -q "Message #" /tmp/callback_detector.log; then
    echo -e "${GREEN}✓ Messages were received${NC}"
    echo "Message count:"
    grep "Message #" /tmp/callback_detector.log | tail -5
else
    echo -e "${RED}✗ No messages received${NC}"
fi

echo -e "\n${YELLOW}Callback detection results:${NC}"
grep -A10 "Detector Summary" /tmp/callback_detector.log || echo "Summary not found"

echo -e "\n${YELLOW}Detected callbacks:${NC}"
grep "SUCCESS.*callback" /tmp/callback_detector.log || echo "No callbacks detected"

echo -e "\n${YELLOW}Full log saved to: /tmp/callback_detector.log${NC}"
