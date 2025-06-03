#!/bin/bash
# test_modified_vulnerable.sh - 수정된 취약 노드 테스트

# 색상 정의
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}=== Modified Vulnerable Subscriber CFI Test ===${NC}"

# 로그 파일
LOG_FILE="/tmp/cfi_test_$(date +%Y%m%d_%H%M%S).log"

# 1. CFI와 함께 노드 실행
echo -e "\n${YELLOW}[1] CFI로 보호된 노드 실행${NC}"
export ROS2_CFI_STRICT=1
export ROS2_CFI_DEBUG=1
export ROS2_CFI_SAVE_STATS=1

LD_PRELOAD=~/ros2_cfi_project/cfi_lib/libday6_final.so \
  ~/ros2_ws/install/basic_communication/lib/basic_communication/simple_cfi_vulnerable \
  2>&1 | tee $LOG_FILE &

NODE_PID=$!
sleep 3

# 2. 정상 메시지 테스트
echo -e "\n${YELLOW}[2] 정상 메시지 전송${NC}"
ros2 topic pub --once /vulnerable_topic std_msgs/msg/String "data: 'Normal message test'" 2>/dev/null
sleep 2

# 로그에서 CFI 등록 확인
if grep -q "Registered new subscription" $LOG_FILE; then
    echo -e "${GREEN}✓ CFI가 subscription을 등록했습니다${NC}"
else
    echo -e "${RED}✗ CFI가 subscription을 등록하지 못했습니다${NC}"
fi

# 3. 공격 실행 (subscription 교체)
echo -e "\n${YELLOW}[3] 공격 페이로드 전송 - subscription 교체${NC}"
ros2 topic pub --once /vulnerable_topic std_msgs/msg/String "data: 'ATTACK:trigger'" 2>/dev/null
sleep 2

# 4. CFI 위반 트리거
echo -e "\n${YELLOW}[4] 다시 메시지 전송 - CFI 위반 예상${NC}"
ros2 topic pub --once /vulnerable_topic std_msgs/msg/String "data: 'This should trigger CFI'" 2>/dev/null
sleep 3

# 5. 결과 분석
echo -e "\n${YELLOW}[5] 결과 분석${NC}"

if grep -q "CFI VIOLATION DETECTED" $LOG_FILE; then
    echo -e "${GREEN}✅ CFI가 공격을 감지했습니다!${NC}"
    echo -e "${GREEN}   Subscription 교체 공격이 차단되었습니다.${NC}"
    RESULT="SUCCESS"
else
    echo -e "${RED}❌ CFI가 공격을 감지하지 못했습니다${NC}"
    
    # 공격 성공 확인
    if [ -f "/tmp/cfi_bypass.txt" ]; then
        echo -e "${RED}   공격이 성공했습니다 - 잘못된 콜백이 실행됨${NC}"
        cat /tmp/cfi_bypass.txt
        rm /tmp/cfi_bypass.txt
    fi
    RESULT="FAILED"
fi

# 프로세스 종료
kill $NODE_PID 2>/dev/null || true

# 6. 상세 로그 출력
echo -e "\n${YELLOW}[6] CFI 로그 요약${NC}"
echo "---"
grep -E "(CFI|callback)" $LOG_FILE | tail -20
echo "---"

echo -e "\n${BLUE}테스트 완료!${NC}"
echo "전체 로그: $LOG_FILE"
echo "결과: $RESULT"

# 환경 변수 정리
unset ROS2_CFI_STRICT
unset ROS2_CFI_DEBUG
unset ROS2_CFI_SAVE_STATS
