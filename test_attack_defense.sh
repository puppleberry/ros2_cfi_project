#!/bin/bash
# test_attack_defense.sh - CFI 공격 방어 테스트

set -e

# 색상 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
MAGENTA='\033[0;35m'
NC='\033[0m'

# 프로젝트 디렉토리
PROJECT_DIR="$HOME/ros2_cfi_project"
CFI_LIB="$PROJECT_DIR/cfi_lib/libday6_final.so"
LOG_DIR="$PROJECT_DIR/evaluation/attack_logs"
mkdir -p $LOG_DIR

echo -e "${MAGENTA}╔════════════════════════════════════════════════╗${NC}"
echo -e "${MAGENTA}║   ROS2 CFI Attack & Defense Demonstration      ║${NC}"
echo -e "${MAGENTA}╚════════════════════════════════════════════════╝${NC}"

# ROS2 환경 설정
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# 1. 취약한 노드 컴파일
echo -e "\n${YELLOW}[1] Building vulnerable target node...${NC}"
cd $PROJECT_DIR/vulnerable_node

# CMakeLists.txt 생성
cat > CMakeLists.txt << 'EOF'
cmake_minimum_required(VERSION 3.8)
project(vulnerable_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(vulnerable_target vulnerable_target.cpp)
ament_target_dependencies(vulnerable_target rclcpp std_msgs)

install(TARGETS
  vulnerable_target
  DESTINATION lib/${PROJECT_NAME})

ament_package()
EOF

# package.xml 생성
cat > package.xml << 'EOF'
<?xml version="1.0"?>
<package format="3">
  <name>vulnerable_node</name>
  <version>0.0.1</version>
  <description>Vulnerable ROS2 node for CFI testing</description>
  <maintainer email="test@test.com">test</maintainer>
  <license>Apache-2.0</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <depend>rclcpp</depend>
  <depend>std_msgs</depend>
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
EOF

# 빌드
cd ~/ros2_ws
colcon build --packages-select vulnerable_node --symlink-install
source install/setup.bash

echo -e "${GREEN}✓ Vulnerable node built successfully${NC}"

# 2. 공격 스크립트 준비
echo -e "\n${YELLOW}[2] Preparing attack script...${NC}"
cd $PROJECT_DIR/exploit
chmod +x attack_demo.py

# 3. CFI 없이 공격 테스트
echo -e "\n${YELLOW}[3] Testing attack WITHOUT CFI protection...${NC}"
echo -e "${RED}⚠️  WARNING: This may crash the vulnerable node${NC}"

# 취약한 노드 실행 (백그라운드)
echo -e "${BLUE}Starting vulnerable node...${NC}"
timeout 15s ~/ros2_ws/install/vulnerable_node/lib/vulnerable_node/vulnerable_target 2>&1 | tee $LOG_DIR/no_cfi_test.log &
TARGET_PID=$!
sleep 2

# 공격 실행
echo -e "${BLUE}Launching attack...${NC}"
python3 $PROJECT_DIR/exploit/attack_demo.py &
ATTACK_PID=$!

# 결과 대기
sleep 8

# 프로세스 상태 확인
if kill -0 $TARGET_PID 2>/dev/null; then
    echo -e "${YELLOW}Target still running (PID: $TARGET_PID)${NC}"
    kill $TARGET_PID 2>/dev/null || true
else
    echo -e "${RED}✓ Target crashed or terminated!${NC}"
fi

kill $ATTACK_PID 2>/dev/null || true
wait $TARGET_PID 2>/dev/null || true
wait $ATTACK_PID 2>/dev/null || true

echo -e "\n${YELLOW}[4] Testing attack WITH CFI protection...${NC}"
echo -e "${GREEN}CFI should detect and prevent the corruption${NC}"

# CFI 환경 변수 설정
export ROS2_CFI_STRICT=1  # Strict 모드 활성화
export ROS2_CFI_DEBUG=1   # 디버그 출력

# CFI와 함께 취약한 노드 실행
echo -e "${BLUE}Starting vulnerable node with CFI...${NC}"
LD_PRELOAD=$CFI_LIB timeout 15s ~/ros2_ws/install/vulnerable_node/lib/vulnerable_node/vulnerable_target 2>&1 | tee $LOG_DIR/with_cfi_test.log &
TARGET_PID=$!
sleep 2

# 공격 실행
echo -e "${BLUE}Launching attack against CFI-protected node...${NC}"
python3 $PROJECT_DIR/exploit/attack_demo.py &
ATTACK_PID=$!

# 결과 대기
sleep 8

# 프로세스 상태 확인
if kill -0 $TARGET_PID 2>/dev/null; then
    echo -e "${GREEN}✓ Target still running with CFI protection (PID: $TARGET_PID)${NC}"
    kill $TARGET_PID 2>/dev/null || true
else
    echo -e "${YELLOW}Target terminated (check if CFI blocked attack)${NC}"
fi

kill $ATTACK_PID 2>/dev/null || true
wait $TARGET_PID 2>/dev/null || true
wait $ATTACK_PID 2>/dev/null || true

# 5. 결과 분석
echo -e "\n${MAGENTA}╔════════════════════════════════════════════════╗${NC}"
echo -e "${MAGENTA}║              Attack Test Results               ║${NC}"
echo -e "${MAGENTA}╚════════════════════════════════════════════════╝${NC}"

echo -e "\n${YELLOW}Without CFI:${NC}"
if grep -q "Segmentation fault\|memory corrupted" $LOG_DIR/no_cfi_test.log; then
    echo -e "${RED}  ✗ Memory corruption detected - Attack succeeded!${NC}"
elif grep -q "Subscription memory after copy" $LOG_DIR/no_cfi_test.log; then
    echo -e "${YELLOW}  ⚠ Overflow occurred, check memory state${NC}"
    grep -A5 "Subscription memory after copy" $LOG_DIR/no_cfi_test.log | head -10
fi

echo -e "\n${YELLOW}With CFI:${NC}"
if grep -q "CFI VIOLATION DETECTED" $LOG_DIR/with_cfi_test.log; then
    echo -e "${GREEN}  ✓ CFI detected the attack!${NC}"
    grep "CFI VIOLATION\|Expected hash\|Current hash" $LOG_DIR/with_cfi_test.log | head -5
fi

if grep -q "BLOCKING EXECUTION\|Execution blocked\|Aborting" $LOG_DIR/with_cfi_test.log; then
    echo -e "${GREEN}  ✓ CFI blocked the malicious execution!${NC}"
fi

# 6. 간단한 수동 테스트 명령어 제공
echo -e "\n${CYAN}For manual testing:${NC}"
echo "1. Run vulnerable node without CFI:"
echo "   $ ~/ros2_ws/install/vulnerable_node/lib/vulnerable_node/vulnerable_target"
echo ""
echo "2. Run vulnerable node with CFI:"
echo "   $ LD_PRELOAD=$CFI_LIB ~/ros2_ws/install/vulnerable_node/lib/vulnerable_node/vulnerable_target"
echo ""
echo "3. Send attack payload:"
echo "   $ ros2 topic pub --once /attack_topic std_msgs/msg/String \"data: 'OVERFLOW:AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA'\""

echo -e "\n${GREEN}✨ Attack defense test completed!${NC}"
echo -e "${YELLOW}Check logs in: $LOG_DIR${NC}"
