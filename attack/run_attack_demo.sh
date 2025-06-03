#!/bin/bash
# run_attack_demo.sh - CFI 공격/방어 시연

# 색상 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

echo -e "${CYAN}╔════════════════════════════════════════════════════════╗${NC}"
echo -e "${CYAN}║         ROS2 CFI Attack/Defense Demonstration          ║${NC}"
echo -e "${CYAN}╚════════════════════════════════════════════════════════╝${NC}"

# ROS2 환경 설정
source /opt/ros/humble/setup.bash

# 시나리오 선택
echo -e "\n${YELLOW}Select demonstration scenario:${NC}"
echo "1. Attack WITHOUT CFI (vulnerable)"
echo "2. Attack WITH CFI (protected)"
echo "3. Both (comparison)"
echo -n "Enter choice (1-3): "
read choice

run_scenario() {
    local with_cfi=$1
    local scenario_name=$2
    
    echo -e "\n${CYAN}=== $scenario_name ===${NC}"
    
    # 노드 시작
    if [ "$with_cfi" = "true" ]; then
        echo -e "${GREEN}Starting node WITH CFI protection...${NC}"
        export ROS2_CFI_DEBUG=1
        export ROS2_CFI_STRICT=1  # Strict 모드 활성화
        LD_PRELOAD=../cfi_lib/libday6_final_always_active.so ./vulnerable_node &
    else
        echo -e "${RED}Starting node WITHOUT CFI protection...${NC}"
        ./vulnerable_node &
    fi
    
    NODE_PID=$!
    sleep 2
    
    # 정상 메시지
    echo -e "\n${YELLOW}[Step 1] Sending normal message...${NC}"
    ros2 topic pub --once /topic std_msgs/msg/String "data: 'Hello World'" 2>/dev/null
    sleep 1
    
    # 공격 메시지 - Subscription 변조
    echo -e "\n${YELLOW}[Step 2] Sending attack message (subscription corruption)...${NC}"
    ros2 topic pub --once /topic std_msgs/msg/String "data: 'OVERFLOW:CORRUPT'" 2>/dev/null
    sleep 1
    
    # CFI 트리거
    echo -e "\n${YELLOW}[Step 3] Triggering next callback...${NC}"
    ros2 topic pub --once /topic std_msgs/msg/String "data: 'Trigger CFI check'" 2>/dev/null
    sleep 2
    
    # 결과 확인
    if kill -0 $NODE_PID 2>/dev/null; then
        if [ "$with_cfi" = "true" ]; then
            echo -e "\n${YELLOW}⚠️  Node still running (CFI in permissive mode?)${NC}"
        else
            echo -e "\n${RED}❌ Attack succeeded! Node still running with corrupted subscription${NC}"
        fi
        kill $NODE_PID 2>/dev/null
    else
        if [ "$with_cfi" = "true" ]; then
            echo -e "\n${GREEN}✅ CFI detected the attack and terminated the process!${NC}"
        else
            echo -e "\n${RED}❌ Node crashed (uncontrolled)${NC}"
        fi
    fi
    
    wait $NODE_PID 2>/dev/null
    unset ROS2_CFI_DEBUG
    unset ROS2_CFI_STRICT
}

# 시나리오 실행
case $choice in
    1)
        run_scenario "false" "Scenario 1: Attack WITHOUT CFI"
        ;;
    2)
        run_scenario "true" "Scenario 2: Attack WITH CFI"
        ;;
    3)
        run_scenario "false" "Scenario 1: Attack WITHOUT CFI"
        echo -e "\n${CYAN}Press Enter to continue with CFI demo...${NC}"
        read
        run_scenario "true" "Scenario 2: Attack WITH CFI"
        ;;
    *)
        echo -e "${RED}Invalid choice!${NC}"
        exit 1
        ;;
esac

echo -e "\n${CYAN}╔════════════════════════════════════════════════════════╗${NC}"
echo -e "${CYAN}║              Demonstration Complete!                   ║${NC}"
echo -e "${CYAN}╚════════════════════════════════════════════════════════╝${NC}"

echo -e "\n${YELLOW}Key Points:${NC}"
echo "• Fast path has been removed to ensure every callback is validated"
echo "• Strict mode enforces immediate termination on violation"
echo "• CFI successfully detects memory corruption in subscription objects"
