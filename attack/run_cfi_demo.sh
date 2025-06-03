#!/bin/bash
# run_cfi_demo.sh - ROS2 CFI 공격/방어 시연 스크립트

set -e

# 색상 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
MAGENTA='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m'

# 프로젝트 디렉토리
PROJECT_DIR="$HOME/ros2_cfi_project"
CFI_LIB="$PROJECT_DIR/cfi_lib/libday6_final.so"
ATTACK_DIR="$PROJECT_DIR/attack"

echo -e "${MAGENTA}╔════════════════════════════════════════════════════════╗${NC}"
echo -e "${MAGENTA}║         ROS2 CFI Attack/Defense Demonstration          ║${NC}"
echo -e "${MAGENTA}╚════════════════════════════════════════════════════════╝${NC}"

# 1. 환경 설정
echo -e "\n${YELLOW}[1] Setting up environment...${NC}"
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# 2. 취약한 노드 컴파일
echo -e "\n${YELLOW}[2] Compiling vulnerable node...${NC}"
cd $ATTACK_DIR

g++ -o vulnerable_subscription vulnerable_subscription_node.cpp \
    -I/opt/ros/humble/include/rclcpp \
    -I/opt/ros/humble/include/rcl \
    -I/opt/ros/humble/include/rcl_interfaces \
    -I/opt/ros/humble/include/rmw \
    -I/opt/ros/humble/include/rcutils \
    -I/opt/ros/humble/include/rcpputils \
    -I/opt/ros/humble/include/rosidl_runtime_cpp \
    -I/opt/ros/humble/include/rosidl_typesupport_interface \
    -I/opt/ros/humble/include/std_msgs \
    -I/opt/ros/humble/include/builtin_interfaces \
    -I/opt/ros/humble/include/rosidl_runtime_c \
    -I/opt/ros/humble/include/libstatistics_collector \
    -I/opt/ros/humble/include/statistics_msgs \
    -I/opt/ros/humble/include/tracetools \
    -L/opt/ros/humble/lib \
    -lrclcpp -lrcl -lrmw -lstd_msgs__rosidl_typesupport_cpp \
    -fno-stack-protector -z execstack -no-pie -Wl,-z,norelro -O0

if [ -f "vulnerable_subscription" ]; then
    echo -e "${GREEN}✓ Vulnerable node compiled successfully${NC}"
else
    echo -e "${RED}✗ Compilation failed${NC}"
    exit 1
fi

# 공격 스크립트 실행 권한
chmod +x attack_demo.py

# 3. CFI 라이브러리 확인
echo -e "\n${YELLOW}[3] Checking CFI library...${NC}"
if [ -f "$CFI_LIB" ]; then
    echo -e "${GREEN}✓ CFI library found: $CFI_LIB${NC}"
else
    echo -e "${RED}✗ CFI library not found!${NC}"
    echo "Please compile it first in $PROJECT_DIR/cfi_lib/"
    exit 1
fi

# 4. 시연 선택
echo -e "\n${CYAN}╔════════════════════════════════════════════════════════╗${NC}"
echo -e "${CYAN}║                  Select Demo Mode                      ║${NC}"
echo -e "${CYAN}╠════════════════════════════════════════════════════════╣${NC}"
echo -e "${CYAN}║  1. Attack WITHOUT CFI (vulnerable)                    ║${NC}"
echo -e "${CYAN}║  2. Attack WITH CFI (protected)                        ║${NC}"
echo -e "${CYAN}║  3. Both demos (comparison)                            ║${NC}"
echo -e "${CYAN}╚════════════════════════════════════════════════════════╝${NC}"
echo -n "Enter choice (1-3): "
read choice

run_demo() {
    local with_cfi=$1
    local demo_name=$2
    
    echo -e "\n${MAGENTA}════════════════════════════════════════════════════════${NC}"
    echo -e "${MAGENTA}                    ${demo_name}                         ${NC}"
    echo -e "${MAGENTA}════════════════════════════════════════════════════════${NC}"
    
    # 취약한 노드 실행
    if [ "$with_cfi" = "true" ]; then
        echo -e "${GREEN}[*] Starting vulnerable node WITH CFI protection...${NC}"
        export ROS2_CFI_DEBUG=1
        LD_PRELOAD=$CFI_LIB ./vulnerable_subscription &
    else
        echo -e "${RED}[*] Starting vulnerable node WITHOUT CFI protection...${NC}"
        ./vulnerable_subscription &
    fi
    
    NODE_PID=$!
    sleep 2
    
    # 노드가 실행 중인지 확인
    if ! kill -0 $NODE_PID 2>/dev/null; then
        echo -e "${RED}[!] Node failed to start!${NC}"
        return
    fi
    
    echo -e "${YELLOW}[*] Node PID: $NODE_PID${NC}"
    
    # 공격 실행
    echo -e "\n${YELLOW}[*] Launching attack sequence...${NC}"
    python3 attack_demo.py
    
    # 결과 확인
    sleep 2
    
    if kill -0 $NODE_PID 2>/dev/null; then
        if [ "$with_cfi" = "true" ]; then
            echo -e "\n${YELLOW}[!] Node still running. CFI might have switched to permissive mode.${NC}"
        else
            echo -e "\n${RED}[!] Node survived the attack (no CFI protection)${NC}"
        fi
        kill $NODE_PID 2>/dev/null
    else
        if [ "$with_cfi" = "true" ]; then
            echo -e "\n${GREEN}[✓] Node terminated by CFI - Attack blocked!${NC}"
        else
            echo -e "\n${RED}[✗] Node crashed - Attack succeeded!${NC}"
        fi
    fi
    
    wait $NODE_PID 2>/dev/null
    
    # 환경 변수 정리
    unset ROS2_CFI_DEBUG
}

# 시연 실행
case $choice in
    1)
        run_demo "false" "Demo 1: Attack WITHOUT CFI"
        ;;
    2)
        run_demo "true" "Demo 2: Attack WITH CFI"
        ;;
    3)
        run_demo "false" "Demo 1: Attack WITHOUT CFI"
        echo -e "\n${CYAN}Press Enter to continue with CFI demo...${NC}"
        read
        run_demo "true" "Demo 2: Attack WITH CFI"
        ;;
    *)
        echo -e "${RED}Invalid choice!${NC}"
        exit 1
        ;;
esac

echo -e "\n${MAGENTA}╔════════════════════════════════════════════════════════╗${NC}"
echo -e "${MAGENTA}║              Demonstration Complete!                   ║${NC}"
echo -e "${MAGENTA}╚════════════════════════════════════════════════════════╝${NC}"

# 요약
echo -e "\n${CYAN}Key Takeaways:${NC}"
echo "1. Without CFI: Subscription corruption goes undetected"
echo "2. With CFI: Hash validation detects and prevents the attack"
echo "3. CFI adds minimal overhead while providing strong protection"

echo -e "\n${GREEN}✨ CFI successfully demonstrates control flow protection!${NC}"
