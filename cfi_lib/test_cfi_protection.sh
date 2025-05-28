#!/bin/bash
# test_cfi_protection.sh - CFI 보호 기능 테스트

set -e

# 색상 정의
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${GREEN}=== ROS2 CFI Protection Test ===${NC}"

cd ~/ros2_cfi_project/cfi_lib

# 1. CFI 라이브러리 빌드
echo -e "\n${YELLOW}Building CFI callback interceptor...${NC}"
g++ -std=c++17 -fPIC -shared -o libcfi_protection.so callback_interceptor.cpp -ldl

# 2. 심볼 확인
echo -e "\n${YELLOW}Checking symbols...${NC}"
echo "Our hooks:"
nm -D libcfi_protection.so | grep -E "create_subscription|execute_subscription" | c++filt

# 3. ROS2 환경 설정
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# 테스트 함수
run_test() {
    local test_name=$1
    local node_name=$2
    local extra_env=$3
    
    echo -e "\n${BLUE}=== Test: $test_name ===${NC}"
    echo "Running $node_name with: $extra_env"
    
    # 로그 초기화
    rm -f /tmp/cfi_test_${test_name}.log
    
    # 노드 실행 (백그라운드)
    (
        cd ~/ros2_ws
        export LD_PRELOAD=~/ros2_cfi_project/cfi_lib/libcfi_protection.so
        eval "export $extra_env"
        timeout 5s ros2 run basic_communication $node_name 2>&1 | tee /tmp/cfi_test_${test_name}.log
    ) &
    
    NODE_PID=$!
    
    # 노드가 준비될 때까지 대기
    sleep 2
    
    # 메시지 발행
    echo "Publishing messages..."
    ros2 topic pub --once /topic std_msgs/msg/String "data: 'Test 1'" 2>/dev/null || true
    sleep 0.5
    ros2 topic pub --once /topic std_msgs/msg/String "data: 'Test 2'" 2>/dev/null || true
    
    # 종료 대기
    wait $NODE_PID 2>/dev/null || true
    
    # 결과 확인
    echo -e "\n${YELLOW}Results for $test_name:${NC}"
    if grep -q "CFI Callback Interceptor Loaded" /tmp/cfi_test_${test_name}.log; then
        echo -e "${GREEN}✓ CFI library loaded${NC}"
    else
        echo -e "${RED}✗ CFI library not loaded${NC}"
    fi
    
    if grep -q "create_subscription for topic" /tmp/cfi_test_${test_name}.log; then
        echo -e "${GREEN}✓ Subscription creation hooked${NC}"
        grep "Registered callback" /tmp/cfi_test_${test_name}.log || true
    fi
    
    if grep -q "execute_subscription: topic" /tmp/cfi_test_${test_name}.log; then
        echo -e "${GREEN}✓ Callback execution hooked${NC}"
        grep "execute_subscription" /tmp/cfi_test_${test_name}.log | tail -3
    fi
    
    # 통계 출력
    echo -e "\n${YELLOW}Statistics:${NC}"
    grep -A5 "Statistics:" /tmp/cfi_test_${test_name}.log || true
}

# 4. 테스트 실행
echo -e "\n${GREEN}=== Running Tests ===${NC}"

# Test 1: 정상 동작 (CFI 활성화)
run_test "normal_cfi_enabled" "subscriber" "ROS2_CFI_DISABLE=0"

# Test 2: CFI 비활성화
run_test "cfi_disabled" "subscriber" "ROS2_CFI_DISABLE=1"

# Test 3: Strict 모드
run_test "strict_mode" "subscriber" "ROS2_CFI_STRICT=1"

# Test 4: 취약한 노드 테스트
echo -e "\n${BLUE}=== Test: Vulnerable Node ===${NC}"
echo "Testing with vulnerable_subscriber..."

# 정상 메시지로 테스트
(
    cd ~/ros2_ws
    export LD_PRELOAD=~/ros2_cfi_project/cfi_lib/libcfi_protection.so
    export ROS2_CFI_STRICT=0
    timeout 10s ros2 run basic_communication vulnerable_subscriber 2>&1 | tee /tmp/cfi_vulnerable_test.log
) &

VULN_PID=$!
sleep 3

# 정상 메시지
echo -e "\n${YELLOW}Sending normal message...${NC}"
ros2 topic pub --once /vulnerable_topic std_msgs/msg/String "data: 'Hello World'" 2>/dev/null || true
sleep 1

# 공격 메시지 (vulnerable_subscriber의 create_attack_payload() 출력 사용)
echo -e "\n${YELLOW}Sending attack message...${NC}"
# 실제 공격 페이로드는 vulnerable_subscriber 실행 시 출력됨
echo "Check the vulnerable_subscriber output for the attack payload"

# 종료 대기
kill $VULN_PID 2>/dev/null || true
wait $VULN_PID 2>/dev/null || true

# 5. 최종 요약
echo -e "\n${GREEN}=== Test Summary ===${NC}"
echo "Log files created:"
ls -la /tmp/cfi_test_*.log /tmp/cfi_vulnerable_test.log 2>/dev/null || true

echo -e "\n${YELLOW}To view detailed logs:${NC}"
echo "cat /tmp/cfi_test_normal_cfi_enabled.log"
echo "cat /tmp/cfi_vulnerable_test.log"

echo -e "\n${GREEN}Test completed!${NC}"
