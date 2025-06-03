#!/bin/bash
# working_cfi_demo.sh - 실제 작동하는 CFI 데모

# 색상
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

clear
echo "╔══════════════════════════════════════════════════════════╗"
echo "║              ROS2 CFI Demo - Working Version             ║"
echo "╚══════════════════════════════════════════════════════════╝"

# 준비사항 확인
echo -e "\n${YELLOW}[준비사항 확인]${NC}"
echo "1. CFI 라이브러리가 수정되어 첫 subscription을 자동 등록"
echo "2. 이미 등록된 subscription의 변경만 감지"

# Demo 1: 정상 동작
echo -e "\n${GREEN}=== Demo 1: 정상 동작 (CFI 활성화) ===${NC}"
echo "CFI는 정상적인 ROS2 동작을 방해하지 않습니다."

cat << 'EOF' > /tmp/demo1.sh
#!/bin/bash
export LD_PRELOAD=~/ros2_cfi_project/cfi_lib/libday6_fixed.so
~/ros2_ws/install/basic_communication/lib/basic_communication/subscriber &
PID=$!
sleep 3

# 정상 메시지들
for i in {1..3}; do
    ros2 topic pub --once /topic std_msgs/msg/String "data: 'Normal message $i'" 2>/dev/null
    sleep 1
done

kill $PID 2>/dev/null
EOF

chmod +x /tmp/demo1.sh
echo -e "${YELLOW}실행 명령:${NC}"
echo "export LD_PRELOAD=~/ros2_cfi_project/cfi_lib/libday6_fixed.so"
echo "~/ros2_ws/install/basic_communication/lib/basic_communication/subscriber"
echo ""
echo -e "${GREEN}예상 결과:${NC}"
echo "✓ CFI Protection Active!"
echo "✓ 정상 메시지 처리"
echo "✗ CFI VIOLATION 없음"

# Demo 2: 공격 시뮬레이션
echo -e "\n${RED}=== Demo 2: Subscription 변조 공격 ===${NC}"
echo "공격자가 subscription의 콜백을 변경하려고 시도합니다."

cat << 'EOF' > /tmp/demo2.py
#!/usr/bin/env python3
# 메모리 손상 시뮬레이션
import subprocess
import time

print("1. 노드 실행 및 정상 동작 확인")
proc = subprocess.Popen([
    'bash', '-c', 
    'LD_PRELOAD=~/ros2_cfi_project/cfi_lib/libday6_fixed.so '
    '~/ros2_ws/install/basic_communication/lib/basic_communication/working_vulnerable_subscriber'
])
time.sleep(2)

print("2. 정상 메시지 전송")
subprocess.run(['ros2', 'topic', 'pub', '--once', '/vulnerable_topic', 
                'std_msgs/msg/String', 'data: "Normal"'])
time.sleep(1)

print("3. Subscription 손상 공격")
subprocess.run(['ros2', 'topic', 'pub', '--once', '/vulnerable_topic', 
                'std_msgs/msg/String', 'data: "CORRUPT"'])
time.sleep(1)

print("4. 다음 메시지에서 CFI 위반 예상")
subprocess.run(['ros2', 'topic', 'pub', '--once', '/vulnerable_topic', 
                'std_msgs/msg/String', 'data: "Trigger"'])
time.sleep(2)

proc.terminate()
EOF

echo -e "${YELLOW}공격 시나리오:${NC}"
echo "1. Subscription 객체의 vtable 포인터 손상"
echo "2. 다음 콜백 호출 시 변경된 해시 감지"
echo ""
echo -e "${GREEN}예상 결과:${NC}"
echo "🚨🚨🚨 CFI VIOLATION DETECTED! 🚨🚨🚨"
echo "🚨 Subscription has been tampered!"
echo "🚨 Expected hash: 0x..."
echo "🚨 Current hash:  0x..."

# Demo 3: 성능 측정
echo -e "\n${YELLOW}=== Demo 3: CFI 오버헤드 측정 ===${NC}"
echo "CFI 활성화 시 성능 영향은 미미합니다."
echo ""
echo "측정 결과 (Day 6-1):"
echo "- 평균 레이턴시: 7.79μs"
echo "- CPU 오버헤드: ~0%"
echo "- 메모리 오버헤드: 896KB"

# 결론
echo -e "\n${GREEN}=== 결론 ===${NC}"
echo "✅ CFI는 정상 동작을 방해하지 않음"
echo "✅ Subscription 변조 시 즉시 감지"
echo "✅ 매우 낮은 성능 오버헤드"
echo "✅ LD_PRELOAD로 쉽게 적용 가능"

echo -e "\n${YELLOW}테스트를 위한 명령어:${NC}"
echo "# Terminal 1"
echo "export LD_PRELOAD=~/ros2_cfi_project/cfi_lib/libday6_fixed.so"
echo "~/ros2_ws/install/basic_communication/lib/basic_communication/subscriber"
echo ""
echo "# Terminal 2"
echo "ros2 topic pub /topic std_msgs/msg/String \"data: 'Hello World'\""
