#!/bin/bash
# direct_cfi_test.sh - 직접적인 CFI 테스트

echo "=== 직접 CFI 테스트 ==="

# 절대 경로 사용
CFI_LIB="$HOME/ros2_cfi_project/cfi_lib/libday6_final.so"
NODE="$HOME/ros2_ws/install/basic_communication/lib/basic_communication/subscriber"

# 1. CFI 라이브러리 존재 확인
echo "[1] CFI 라이브러리 확인"
if [ ! -f "$CFI_LIB" ]; then
    echo "ERROR: CFI 라이브러리를 찾을 수 없습니다: $CFI_LIB"
    exit 1
fi
ls -la "$CFI_LIB"

# 2. 노드 존재 확인
echo -e "\n[2] 노드 확인"
if [ ! -f "$NODE" ]; then
    echo "ERROR: 노드를 찾을 수 없습니다: $NODE"
    exit 1
fi
ls -la "$NODE"

# 3. CFI 활성화 테스트
echo -e "\n[3] CFI 활성화 테스트"
export LD_PRELOAD="$CFI_LIB"
export ROS2_CFI_DEBUG=1

# 간단한 명령으로 CFI 로드 확인
echo "LD_PRELOAD 설정: $LD_PRELOAD"
echo "테스트 실행:"

# subscriber 노드 실행
timeout 5s $NODE 2>&1 | grep -E "(CFI|SUCCESS)" || echo "CFI 로그가 없습니다"

echo -e "\n[4] 디버깅 정보"
echo "ldd 출력:"
ldd $NODE | grep -E "(librclcpp|libdl)"

echo -e "\n[5] 프로세스 맵 확인"
# 프로세스 실행하고 maps 확인
$NODE &
PID=$!
sleep 2
if [ -d "/proc/$PID" ]; then
    echo "프로세스 $PID의 메모리 맵:"
    cat /proc/$PID/maps | grep -E "(libday6|librclcpp)" | head -5
fi
kill $PID 2>/dev/null

echo -e "\n완료"
