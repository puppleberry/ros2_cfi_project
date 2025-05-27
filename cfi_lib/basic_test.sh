#!/bin/bash
# basic_test.sh - 가장 기본적인 후킹 테스트

set -e

# 색상 정의
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${GREEN}=== Basic Hook Test ===${NC}"

# 1. 간단한 라이브러리 빌드
echo -e "\n${YELLOW}Building simple hook library...${NC}"
g++ -std=c++17 -fPIC -shared -o libsimple_cfi.so simple_hook_test.cpp -ldl

# 2. 심볼 확인
echo -e "\n${YELLOW}Checking symbols...${NC}"
echo "Our library symbols:"
nm -D libsimple_cfi.so | grep create_subscription || echo "No symbols found"

echo -e "\nROS2 library symbols:"
nm -D /opt/ros/humble/lib/librclcpp.so | grep "NodeTopics19create_subscription" | head -3

# 3. 테스트 실행
echo -e "\n${YELLOW}Running test...${NC}"

# ROS2 환경 설정
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# 간단한 테스트 - LD_DEBUG로 라이브러리 로딩 확인
echo -e "\n${YELLOW}Test 1: Library loading test${NC}"
LD_DEBUG=libs LD_PRELOAD=./libsimple_cfi.so ros2 topic list 2>&1 | grep -E "libsimple_cfi|init.*CFI" | head -10

# 실제 노드 테스트
echo -e "\n${YELLOW}Test 2: Subscriber node test${NC}"
rm -f /tmp/hook_test.log
LD_PRELOAD=./libsimple_cfi.so timeout 3s ros2 run basic_communication subscriber 2>&1 | tee /tmp/hook_test.log

# 결과 확인
echo -e "\n${YELLOW}Checking results...${NC}"
if grep -q "Simple ROS2 CFI Library Loaded" /tmp/hook_test.log; then
    echo -e "${GREEN}✓ Library loaded successfully${NC}"
else
    echo -e "${RED}✗ Library loading failed${NC}"
fi

if grep -q "HOOKED: create_subscription called" /tmp/hook_test.log; then
    echo -e "${GREEN}✓ Hook successful!${NC}"
else
    echo -e "${RED}✗ Hook failed${NC}"
fi

echo -e "\n${YELLOW}Test completed. Check /tmp/hook_test.log for details.${NC}"
