#!/bin/bash
# test_callback_analysis.sh - 콜백 호출 경로 분석

set -e

# 색상 정의
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${GREEN}=== Callback Analysis Test ===${NC}"

cd ~/ros2_cfi_project/cfi_lib

# 1. 분석 라이브러리 빌드
echo -e "\n${YELLOW}Building analysis library...${NC}"
g++ -std=c++17 -fPIC -shared -o libcallback_analysis.so callback_hook_analysis.cpp -ldl

# 2. 심볼 확인
echo -e "\n${YELLOW}Checking our hooks...${NC}"
nm -D libcallback_analysis.so | grep -E "handle_message|execute_subscription" || echo "No symbols found"

# 3. ROS2 환경 설정
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# 4. 분석 실행
echo -e "\n${YELLOW}Running subscriber with analysis hooks...${NC}"
echo "Starting subscriber node (will run for 5 seconds)..."

# 로그 파일 초기화
rm -f /tmp/callback_analysis.log

# subscriber 실행 (백그라운드)
(
    cd ~/ros2_ws
    LD_PRELOAD=~/ros2_cfi_project/cfi_lib/libcallback_analysis.so \
    timeout 5s ros2 run basic_communication subscriber 2>&1 | tee /tmp/callback_analysis.log
) &

SUB_PID=$!

# 잠시 대기 후 메시지 발송
sleep 2

echo -e "\n${YELLOW}Publishing test messages...${NC}"
ros2 topic pub --once /topic std_msgs/msg/String "data: 'Test message 1'"
sleep 0.5
ros2 topic pub --once /topic std_msgs/msg/String "data: 'Test message 2'"
sleep 0.5
ros2 topic pub --once /topic std_msgs/msg/String "data: 'Test message 3'"

# 프로세스 종료 대기
wait $SUB_PID 2>/dev/null || true

# 결과 분석
echo -e "\n${YELLOW}Analysis Results:${NC}"
if grep -q "HOOKED: SubscriptionBase::handle_message" /tmp/callback_analysis.log; then
    echo -e "${GREEN}✓ handle_message hook successful${NC}"
    echo -e "\n${YELLOW}Stack traces found:${NC}"
    grep -A20 "Stack Trace" /tmp/callback_analysis.log | head -40
else
    echo -e "${RED}✗ handle_message not hooked${NC}"
fi

if grep -q "HOOKED: Executor::execute_subscription" /tmp/callback_analysis.log; then
    echo -e "${GREEN}✓ execute_subscription hook successful${NC}"
else
    echo -e "${RED}✗ execute_subscription not hooked${NC}"
fi

echo -e "\n${YELLOW}Full log saved to: /tmp/callback_analysis.log${NC}"
