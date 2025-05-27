#!/bin/bash
# build_and_test.sh - CFI 라이브러리 빌드 및 테스트

set -e  # 오류 발생 시 중단

# 색상 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${GREEN}=== Building CFI Library ===${NC}"

# 1. 기존 라이브러리 빌드
echo -e "\n${YELLOW}[1] Building main CFI library...${NC}"
make clean
make libros2_cfi.so

# 2. 간단한 테스트 라이브러리 빌드
echo -e "\n${YELLOW}[2] Building simple test library...${NC}"
g++ -std=c++17 -fPIC -shared -o libsimple_cfi.so simple_hook_test.cpp -ldl

echo -e "\n${GREEN}=== Testing Phase ===${NC}"

# 3. 간단한 라이브러리로 먼저 테스트
echo -e "\n${YELLOW}[3] Testing with simple library...${NC}"
echo "Running subscriber with simple CFI..."

# 로그 초기화
rm -f /tmp/simple_cfi_test.log

# ROS2 환경 설정
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# 테스트 실행 (5초 타임아웃)
echo -e "${YELLOW}Starting subscriber node...${NC}"
(
    cd ~/ros2_ws
    export LD_PRELOAD=~/ros2_cfi_project/cfi_lib/libsimple_cfi.so
    timeout 5s bash -c "ros2 run basic_communication subscriber 2>&1 | tee /tmp/simple_cfi_test.log"
) || true

# 결과 확인
echo -e "\n${YELLOW}[4] Checking results...${NC}"
if grep -q "HOOKED: create_subscription called" /tmp/simple_cfi_test.log; then
    echo -e "${GREEN}✓ Hook successful! create_subscription was intercepted.${NC}"
    echo -e "\n${YELLOW}Hook output:${NC}"
    grep -A5 "HOOKED:" /tmp/simple_cfi_test.log
else
    echo -e "${RED}✗ Hook failed. Function was not intercepted.${NC}"
    echo -e "\n${YELLOW}Debug: Checking library symbols...${NC}"
    nm -D libsimple_cfi.so | grep create_subscription || echo "No create_subscription symbol found"
    
    echo -e "\n${YELLOW}Debug: Checking if library was loaded...${NC}"
    grep "CFI Library Loaded" /tmp/simple_cfi_test.log || echo "Library was not loaded"
fi

# 5. 전체 테스트 옵션
echo -e "\n${YELLOW}[5] Full library test options:${NC}"
echo "To test with full CFI library:"
echo "  LD_PRELOAD=./libros2_cfi.so ros2 run basic_communication subscriber"
echo ""
echo "To test with vulnerable node:"
echo "  LD_PRELOAD=./libros2_cfi.so ros2 run basic_communication vulnerable_subscriber"
echo ""
echo "To monitor logs:"
echo "  tail -f /tmp/ros2_cfi.log"

# 6. 심볼 비교
echo -e "\n${YELLOW}[6] Symbol comparison:${NC}"
echo "Symbols in our library:"
nm -D libsimple_cfi.so | grep "_ZN6rclcpp" | head -5

echo -e "\nSymbols in ROS2 library:"
nm -D /opt/ros/humble/lib/librclcpp.so | grep "NodeTopics19create_subscription" | head -5
