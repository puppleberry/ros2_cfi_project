#!/bin/bash
# debug_address.sh - CFI 주소 디버깅 테스트

# 색상 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

echo -e "${CYAN}=== CFI Address Debug Test ===${NC}"

# ROS2 환경 설정
source /opt/ros/humble/setup.bash

# 1. CFI 라이브러리 빌드
echo -e "\n${YELLOW}Building CFI with address debug...${NC}"
cd ~/ros2_cfi_project/cfi_lib
g++ -std=c++17 -fPIC -shared -o libday6_final_always_active.so day6_final_cfi_always_active.cpp -ldl -pthread -O2

# 2. 노드 실행
cd ~/ros2_cfi_project/attack

echo -e "\n${YELLOW}Starting vulnerable node with address debugging...${NC}"
export ROS2_CFI_DEBUG=1
export ROS2_CFI_ADDRESS_DEBUG=1
export ROS2_CFI_STRICT=1

LD_PRELOAD=../cfi_lib/libday6_final_always_active.so ./vulnerable_node &
NODE_PID=$!

sleep 2

# 3. 메시지 전송
echo -e "\n${YELLOW}Sending normal message...${NC}"
ros2 topic pub --once /topic std_msgs/msg/String "data: 'Hello'" 2>/dev/null

sleep 1

echo -e "\n${YELLOW}Sending corruption message...${NC}"
ros2 topic pub --once /topic std_msgs/msg/String "data: 'OVERFLOW:CORRUPT'" 2>/dev/null

sleep 1

echo -e "\n${YELLOW}Triggering CFI check...${NC}"
ros2 topic pub --once /topic std_msgs/msg/String "data: 'Trigger'" 2>/dev/null

sleep 2

# 프로세스 종료
kill $NODE_PID 2>/dev/null
wait $NODE_PID 2>/dev/null

echo -e "\n${CYAN}=== Debug complete ===${NC}"
echo -e "${YELLOW}Check the logs above for:${NC}"
echo "1. shared_ptr structure"
echo "2. Extracted vs actual subscription addresses"
echo "3. Hash calculation for bytes 100-109"
echo "4. Whether corruption is detected"
