#!/bin/bash
# test_hook.sh - ROS2 subscription 후킹 테스트

# 색상 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

CFI_LIB_PATH=~/ros2_cfi_project/cfi_lib/libros2_cfi.so

echo -e "${GREEN}=== ROS2 CFI Hook Test ===${NC}"

# 1. 환경 설정
echo -e "\n${YELLOW}Setting up ROS2 environment...${NC}"
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# 2. 로그 파일 초기화
echo -e "${YELLOW}Cleaning log file...${NC}"
rm -f /tmp/ros2_cfi.log
touch /tmp/ros2_cfi.log

# 3. 후킹 테스트를 위한 함수
test_with_node() {
    local node_name=$1
    echo -e "\n${BLUE}Testing with $node_name...${NC}"
    
    # 별도 터미널에서 로그 모니터링 (백그라운드)
    gnome-terminal --title="CFI Log Monitor" -- bash -c "tail -f /tmp/ros2_cfi.log; read" 2>/dev/null || \
    xterm -title "CFI Log Monitor" -e "tail -f /tmp/ros2_cfi.log; read" 2>/dev/null &
    
    # 노드 실행 (CFI 활성화)
    echo -e "${YELLOW}Starting $node_name with CFI...${NC}"
    LD_PRELOAD=$CFI_LIB_PATH ROS2_CFI_LOG_LEVEL=DEBUG timeout 10s ros2 run basic_communication $node_name
    
    echo -e "${GREEN}Test completed. Check log for results.${NC}"
}

# 4. 메뉴
while true; do
    echo -e "\n${GREEN}=== CFI Hook Test Menu ===${NC}"
    echo "1. Test with subscriber node"
    echo "2. Test with vulnerable_subscriber node"
    echo "3. Test with memory_analysis node"
    echo "4. View log file"
    echo "5. Clear log file"
    echo "6. Check hooked functions (nm)"
    echo "7. Exit"
    
    read -p "Select option: " choice
    
    case $choice in
        1)
            test_with_node "subscriber"
            ;;
        2)
            test_with_node "vulnerable_subscriber"
            ;;
        3)
            test_with_node "memory_analysis"
            ;;
        4)
            echo -e "\n${YELLOW}=== CFI Log Contents ===${NC}"
            cat /tmp/ros2_cfi.log | tail -50
            ;;
        5)
            rm -f /tmp/ros2_cfi.log
            touch /tmp/ros2_cfi.log
            echo -e "${GREEN}Log file cleared${NC}"
            ;;
        6)
            echo -e "\n${YELLOW}=== Checking hooked functions ===${NC}"
            nm -D $CFI_LIB_PATH | grep -E "create_subscription|NodeTopics"
            echo -e "\n${YELLOW}=== Original ROS2 functions ===${NC}"
            nm -D /opt/ros/humble/lib/librclcpp.so | grep "NodeTopics19create_subscription" | head -5
            ;;
        7)
            echo -e "${GREEN}Exiting...${NC}"
            exit 0
            ;;
        *)
            echo -e "${RED}Invalid option${NC}"
            ;;
    esac
done
