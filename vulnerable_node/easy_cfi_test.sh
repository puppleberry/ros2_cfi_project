#!/bin/bash
# easy_cfi_test.sh - 5분 안에 CFI 작동을 확인하는 가장 간단한 방법

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
MAGENTA='\033[0;35m'
NC='\033[0m'

echo -e "${MAGENTA}╔════════════════════════════════════════════════╗${NC}"
echo -e "${MAGENTA}║         Easy CFI Test (5 minutes)              ║${NC}"
echo -e "${MAGENTA}╚════════════════════════════════════════════════╝${NC}"

# 경로 설정
CFI_DIR="$HOME/ros2_cfi_project/cfi_lib"
VULN_DIR="$HOME/ros2_cfi_project/vulnerable_node"
CFI_LIB="$CFI_DIR/libcfi_day6_clean.so"

# ROS2 환경
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# 1. CFI 라이브러리 확인
echo -e "\n${YELLOW}[1/4] Checking CFI library...${NC}"
if [ ! -f "$CFI_LIB" ]; then
    echo "Compiling CFI..."
    cd $CFI_DIR
    g++ -std=c++17 -fPIC -shared -o libcfi_day6_clean.so day6_final_cfi.cpp -ldl -pthread -O2
fi
echo -e "${GREEN}✓ CFI ready${NC}"

# 2. 최종 공격 데모 컴파일
echo -e "\n${YELLOW}[2/4] Building attack demo...${NC}"
cd $VULN_DIR

# 간단한 버전으로 컴파일 시도
g++ -o final_attack final_working_attack.cpp \
    -I/opt/ros/humble/include/rclcpp \
    -I/opt/ros/humble/include/std_msgs \
    -I/opt/ros/humble/include/rcl \
    -I/opt/ros/humble/include/rcutils \
    -I/opt/ros/humble/include/rmw \
    -I/opt/ros/humble/include/rcl_interfaces \
    -I/opt/ros/humble/include/rosidl_runtime_c \
    -I/opt/ros/humble/include/rosidl_typesupport_interface \
    -I/opt/ros/humble/include/rcpputils \
    -I/opt/ros/humble/include/builtin_interfaces \
    -I/opt/ros/humble/include/rosidl_runtime_cpp \
    -I/opt/ros/humble/include/tracetools \
    -I/opt/ros/humble/include/rcl_yaml_param_parser \
    -I/opt/ros/humble/include/libyaml_vendor \
    -I/opt/ros/humble/include/statistics_msgs \
    -I/opt/ros/humble/include/libstatistics_collector \
    -L/opt/ros/humble/lib \
    -lrclcpp -lstd_msgs__rosidl_typesupport_cpp \
    -Wl,-rpath,/opt/ros/humble/lib 2>/dev/null || {
    echo -e "${YELLOW}Using existing vulnerable_target instead${NC}"
}

# 3. 테스트 실행
echo -e "\n${YELLOW}[3/4] Running tests...${NC}"

# Test 1: CFI 활성화로 정상 작동 확인
echo -e "\n${BLUE}Test 1: Normal operation with CFI${NC}"
export ROS2_CFI_DEBUG=1
unset ROS2_CFI_STRICT  # 일단 permissive 모드

if [ -f "./final_attack" ]; then
    TARGET="./final_attack"
    TOPIC="/attack_topic"
    MSG1="CHECK"
    MSG2="CORRUPT"
    MSG3="TEST"
else
    TARGET="$HOME/ros2_ws/install/vulnerable_node/lib/vulnerable_node/vulnerable_target"
    TOPIC="/attack_topic"
    MSG1="Hello"
    MSG2="OVERFLOW:AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"
    MSG3="TRIGGER"
fi

echo "Starting node with CFI..."
LD_PRELOAD=$CFI_LIB timeout 15s $TARGET 2>&1 | tee /tmp/cfi_test.log &
PID=$!
sleep 3

echo -e "\n${BLUE}Sending messages...${NC}"
echo "1. Normal message:"
ros2 topic pub --once $TOPIC std_msgs/msg/String "data: '$MSG1'" 2>/dev/null
sleep 2

echo "2. Corruption trigger:"
ros2 topic pub --once $TOPIC std_msgs/msg/String "data: '$MSG2'" 2>/dev/null
sleep 2

echo "3. Test corrupted callback:"
ros2 topic pub --once $TOPIC std_msgs/msg/String "data: '$MSG3'" 2>/dev/null
sleep 2

# 한번 더 시도
ros2 topic pub --once $TOPIC std_msgs/msg/String "data: '$MSG3'" 2>/dev/null
sleep 2

kill $PID 2>/dev/null || true
wait $PID 2>/dev/null || true

# 4. 결과 분석
echo -e "\n${YELLOW}[4/4] Results:${NC}"

echo -e "\n${BLUE}Key events from log:${NC}"
if grep -q "CFI initialized successfully" /tmp/cfi_test.log; then
    echo -e "${GREEN}✓ CFI loaded${NC}"
fi

if grep -q "Registered new subscription" /tmp/cfi_test.log; then
    echo -e "${GREEN}✓ Subscription registered${NC}"
    grep "Registered new subscription" /tmp/cfi_test.log | head -1
fi

if grep -q "CFI VIOLATION DETECTED" /tmp/cfi_test.log; then
    echo -e "${GREEN}✓ CFI detected attack!${NC}"
    echo -e "${YELLOW}Attack was blocked:${NC}"
    grep -A2 "CFI VIOLATION" /tmp/cfi_test.log | head -3
elif grep -q "CORRUPT" /tmp/cfi_test.log; then
    echo -e "${YELLOW}⚠ Corruption was triggered but CFI might not have detected it${NC}"
    echo "This could be because:"
    echo "- The corruption didn't affect the monitored memory region"
    echo "- The attack needs to be more targeted"
else
    echo -e "${RED}✗ Test might not have run correctly${NC}"
fi

# 최종 요약
echo -e "\n${MAGENTA}╔════════════════════════════════════════════════╗${NC}"
echo -e "${MAGENTA}║                Summary                         ║${NC}"
echo -e "${MAGENTA}╚════════════════════════════════════════════════╝${NC}"

if [ -f "./final_attack" ]; then
    echo -e "\n${GREEN}For a clearer demo, run manually:${NC}"
    echo -e "\n${YELLOW}Terminal 1:${NC}"
    echo "cd $VULN_DIR"
    echo "export ROS2_CFI_DEBUG=1"
    echo "export ROS2_CFI_STRICT=1"
    echo "LD_PRELOAD=$CFI_LIB ./final_attack"
    
    echo -e "\n${YELLOW}Terminal 2:${NC}"
    echo "ros2 topic pub --once /attack_topic std_msgs/msg/String \"data: 'CHECK'\""
    echo "ros2 topic pub --once /attack_topic std_msgs/msg/String \"data: 'CORRUPT'\""
    echo "ros2 topic pub --once /attack_topic std_msgs/msg/String \"data: 'TEST'\""
    
    echo -e "\n${GREEN}Expected:${NC}"
    echo "1. CHECK: Shows normal memory state"
    echo "2. CORRUPT: Shows corrupted memory"
    echo "3. TEST: CFI blocks execution with violation message"
else
    echo -e "\n${YELLOW}The original vulnerable_target buffer overflow doesn't reach the subscription.${NC}"
    echo "This is why CFI doesn't detect it - there's no actual corruption of the subscription!"
    echo ""
    echo -e "${GREEN}To see CFI in action, we need an attack that actually corrupts the subscription.${NC}"
fi

echo -e "\n${GREEN}✨ Test complete!${NC}"
