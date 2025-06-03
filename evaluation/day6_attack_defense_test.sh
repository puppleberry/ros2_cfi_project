#!/bin/bash
# day6_attack_defense_test.sh - CFI 공격 방어 테스트 스크립트

set -e

# 색상 정의
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
MAGENTA='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m'

# 로그 디렉토리
LOG_DIR="$HOME/ros2_cfi_project/evaluation/day6_attack_logs"
mkdir -p $LOG_DIR
TIMESTAMP=$(date +%Y%m%d_%H%M%S)

echo -e "${MAGENTA}╔════════════════════════════════════════════════╗${NC}"
echo -e "${MAGENTA}║    ROS2 CFI Day 6-2 - Attack Defense Test     ║${NC}"
echo -e "${MAGENTA}╚════════════════════════════════════════════════╝${NC}"

cd ~/ros2_cfi_project

# ROS2 환경 설정
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# CFI 라이브러리 경로
CFI_LIB="$HOME/ros2_cfi_project/cfi_lib/libday6_final.so"

# 공격 결과 통계
TOTAL_ATTACKS=0
BLOCKED_ATTACKS=0
FAILED_ATTACKS=0

# 공격 테스트 함수
run_attack_test() {
    local attack_name=$1
    local attack_payload=$2
    local expected_result=$3  # "blocked" or "detected"
    
    echo -e "\n${YELLOW}[Attack: $attack_name]${NC}"
    echo -e "${BLUE}Payload: $attack_payload${NC}"
    
    # 취약한 노드 실행 (CFI 활성화)
    export ROS2_CFI_SAVE_STATS=1
    export ROS2_CFI_STATS_FILE="$LOG_DIR/${attack_name}_stats_${TIMESTAMP}.txt"
    
    # Strict 모드로 실행
    export ROS2_CFI_STRICT=1
    
    # 노드 시작
    LD_PRELOAD=$CFI_LIB timeout 10s ~/ros2_ws/install/basic_communication/lib/basic_communication/vulnerable_subscriber \
        2>&1 | tee "$LOG_DIR/${attack_name}_${TIMESTAMP}.log" &
    NODE_PID=$!
    
    # 노드가 준비될 때까지 대기
    sleep 2
    
    # 공격 실행
    echo -e "${RED}Launching attack...${NC}"
    ros2 topic pub --once /vulnerable_topic std_msgs/msg/String "data: '$attack_payload'" 2>/dev/null
    
    # 결과 대기
    sleep 3
    
    # 프로세스 상태 확인
    if kill -0 $NODE_PID 2>/dev/null; then
        # 프로세스가 살아있음
        echo -e "${YELLOW}Process still running - checking CFI detection${NC}"
        kill -TERM $NODE_PID 2>/dev/null
        wait $NODE_PID 2>/dev/null || true
    else
        # 프로세스가 종료됨 (CFI가 차단함)
        echo -e "${GREEN}✓ Process terminated by CFI${NC}"
    fi
    
    # 로그 분석
    if grep -q "CFI VIOLATION DETECTED" "$LOG_DIR/${attack_name}_${TIMESTAMP}.log"; then
        echo -e "${GREEN}✓ CFI detected the attack!${NC}"
        ((BLOCKED_ATTACKS++))
    elif grep -q "악성 함수가 호출되었습니다" "$LOG_DIR/${attack_name}_${TIMESTAMP}.log"; then
        echo -e "${RED}✗ Attack succeeded - malicious function executed${NC}"
        ((FAILED_ATTACKS++))
    elif grep -q "공격 성공!" "$LOG_DIR/${attack_name}_${TIMESTAMP}.log"; then
        echo -e "${RED}✗ Attack succeeded - system compromised${NC}"
        ((FAILED_ATTACKS++))
    else
        echo -e "${YELLOW}⚠ Unclear result - checking logs${NC}"
    fi
    
    ((TOTAL_ATTACKS++))
    
    # 환경 변수 정리
    unset ROS2_CFI_SAVE_STATS
    unset ROS2_CFI_STATS_FILE
    unset ROS2_CFI_STRICT
    
    sleep 2
}

# 1. 기본 Buffer Overflow 공격
echo -e "\n${CYAN}=== Test 1: Basic Buffer Overflow Attack ===${NC}"
# 32바이트 버퍼 + 악성 함수 포인터
BASIC_OVERFLOW="AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA$(printf '\x00\x00\x00\x00\x00\x00\x00\x00')"
run_attack_test "basic_overflow" "$BASIC_OVERFLOW" "blocked"

# 2. 정확한 함수 포인터 덮어쓰기 공격
echo -e "\n${CYAN}=== Test 2: Precise Function Pointer Overwrite ===${NC}"
# vulnerable_subscriber의 create_attack_payload() 함수가 생성하는 페이로드 사용
ATTACK_PAYLOAD="HEX:4141414141414141414141414141414141414141414141414141414141414141"
# 실제 악성 함수 주소를 추가해야 하지만, 먼저 간단한 테스트
run_attack_test "precise_overwrite" "$ATTACK_PAYLOAD" "blocked"

# 3. NOP 슬레드를 포함한 공격
echo -e "\n${CYAN}=== Test 3: NOP Sled Attack ===${NC}"
NOP_SLED="HEX:9090909090909090909090909090909090909090909090909090909090909090"
run_attack_test "nop_sled" "$NOP_SLED" "blocked"

# 4. 부분적 덮어쓰기 공격
echo -e "\n${CYAN}=== Test 4: Partial Overwrite Attack ===${NC}"
# 함수 포인터의 하위 바이트만 변경
PARTIAL_OVERWRITE="HEX:41414141414141414141414141414141414141414141414141414141414141410000"
run_attack_test "partial_overwrite" "$PARTIAL_OVERWRITE" "blocked"

# 5. 다중 공격 시도
echo -e "\n${CYAN}=== Test 5: Multiple Attack Attempts ===${NC}"
for i in {1..5}; do
    echo -e "${BLUE}Attack attempt $i/5${NC}"
    RANDOM_OVERFLOW=$(python3 -c "import random; print(''.join(random.choice('ABCDEF0123456789') for _ in range(48)))")
    run_attack_test "multiple_attempt_$i" "$RANDOM_OVERFLOW" "blocked"
done

# 6. Permissive 모드 테스트
echo -e "\n${CYAN}=== Test 6: Permissive Mode Test ===${NC}"
unset ROS2_CFI_STRICT  # Permissive 모드
echo -e "${YELLOW}Testing with CFI in PERMISSIVE mode${NC}"

LD_PRELOAD=$CFI_LIB timeout 10s ~/ros2_ws/install/basic_communication/lib/basic_communication/vulnerable_subscriber \
    2>&1 | tee "$LOG_DIR/permissive_test_${TIMESTAMP}.log" &
NODE_PID=$!
sleep 2

ros2 topic pub --once /vulnerable_topic std_msgs/msg/String "data: '$BASIC_OVERFLOW'" 2>/dev/null
sleep 3

if kill -0 $NODE_PID 2>/dev/null; then
    echo -e "${YELLOW}Process survived in permissive mode${NC}"
    kill -TERM $NODE_PID 2>/dev/null
else
    echo -e "${RED}Process crashed even in permissive mode${NC}"
fi

# 7. Shadow Stack 우회 시도
echo -e "\n${CYAN}=== Test 7: Shadow Stack Bypass Attempt ===${NC}"
export ROS2_CFI_NO_SHADOW_STACK=1
run_attack_test "no_shadow_stack" "$BASIC_OVERFLOW" "detected"
unset ROS2_CFI_NO_SHADOW_STACK

# 8. 정상 메시지 후 공격 (CFI 상태 확인)
echo -e "\n${CYAN}=== Test 8: Normal Messages Then Attack ===${NC}"
LD_PRELOAD=$CFI_LIB ~/ros2_ws/install/basic_communication/lib/basic_communication/vulnerable_subscriber \
    2>&1 | tee "$LOG_DIR/normal_then_attack_${TIMESTAMP}.log" &
NODE_PID=$!
sleep 2

# 정상 메시지들
for i in {1..3}; do
    echo -e "${BLUE}Sending normal message $i${NC}"
    ros2 topic pub --once /vulnerable_topic std_msgs/msg/String "data: 'Normal message $i'" 2>/dev/null
    sleep 1
done

# 공격 메시지
echo -e "${RED}Sending attack message${NC}"
ros2 topic pub --once /vulnerable_topic std_msgs/msg/String "data: '$BASIC_OVERFLOW'" 2>/dev/null
sleep 2

kill -TERM $NODE_PID 2>/dev/null || true

# 결과 요약
echo -e "\n${MAGENTA}╔════════════════════════════════════════════════╗${NC}"
echo -e "${MAGENTA}║           Attack Defense Test Summary          ║${NC}"
echo -e "${MAGENTA}╚════════════════════════════════════════════════╝${NC}"

echo -e "\n${CYAN}Attack Statistics:${NC}"
echo -e "Total attacks attempted: $TOTAL_ATTACKS"
echo -e "${GREEN}Successfully blocked: $BLOCKED_ATTACKS${NC}"
echo -e "${RED}Failed to block: $FAILED_ATTACKS${NC}"

if [ $BLOCKED_ATTACKS -gt 0 ]; then
    DEFENSE_RATE=$(( BLOCKED_ATTACKS * 100 / TOTAL_ATTACKS ))
    echo -e "\n${GREEN}✅ CFI Defense Rate: ${DEFENSE_RATE}%${NC}"
else
    echo -e "\n${RED}❌ CFI failed to block attacks${NC}"
fi

# 로그 파일 위치
echo -e "\n${YELLOW}Log files saved to: $LOG_DIR${NC}"
echo -e "${YELLOW}Statistics files:${NC}"
ls -la $LOG_DIR/*stats*.txt 2>/dev/null || echo "No stats files found"

# CFI violations 통계
echo -e "\n${CYAN}CFI Violation Details:${NC}"
grep -h "CFI VIOLATION DETECTED" $LOG_DIR/*${TIMESTAMP}.log 2>/dev/null | head -5 || echo "No violations logged"

echo -e "\n${GREEN}✨ Day 6-2 Attack Defense Testing Complete!${NC}"
