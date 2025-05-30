#!/bin/bash
# build_and_test_day6.sh - Day 6 노드 빌드 및 테스트 실행

set -e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}╔════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║    Day 6 - Build and Test Script       ║${NC}"
echo -e "${BLUE}╚════════════════════════════════════════╝${NC}"

# 1. 새 노드 파일 복사
echo -e "\n${YELLOW}[1] Copying new node files...${NC}"
cd ~/ros2_cfi_project

# basic_communication 패키지 디렉토리 확인
PACKAGE_DIR=~/ros2_ws/src/basic_communication
if [ ! -d "$PACKAGE_DIR" ]; then
    echo -e "${RED}Error: Package directory not found!${NC}"
    echo "Expected: $PACKAGE_DIR"
    exit 1
fi

# 새 노드 파일들 복사 (artifacts에서 생성한 파일들)
# cp multi_topic_node.cpp $PACKAGE_DIR/src/
# cp service_test_node.cpp $PACKAGE_DIR/src/
# cp performance_monitor.cpp $PACKAGE_DIR/src/

# CMakeLists.txt 업데이트
# cp CMakeLists.txt $PACKAGE_DIR/

echo -e "${GREEN}✓ Files copied successfully${NC}"

# 2. 패키지 빌드
echo -e "\n${YELLOW}[2] Building ROS2 package...${NC}"
cd ~/ros2_ws
source /opt/ros/humble/setup.bash

colcon build --packages-select basic_communication --symlink-install

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Build successful${NC}"
else
    echo -e "${RED}✗ Build failed${NC}"
    exit 1
fi

# 3. CFI 라이브러리 빌드
echo -e "\n${YELLOW}[3] Building CFI library...${NC}"
cd ~/ros2_cfi_project/cfi_lib

g++ -std=c++17 -fPIC -shared -o libday6_cfi.so day5_working_cfi.cpp -ldl -pthread -O2

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ CFI library built${NC}"
else
    echo -e "${RED}✗ CFI build failed${NC}"
    exit 1
fi

# 4. 환경 설정
source ~/ros2_ws/install/setup.bash

# 5. 바이너리 확인
echo -e "\n${YELLOW}[4] Verifying binaries...${NC}"
BIN_DIR=~/ros2_ws/install/basic_communication/lib/basic_communication

for node in subscriber vulnerable_subscriber benchmark_node multi_topic_node service_test_node performance_monitor; do
    if [ -f "$BIN_DIR/$node" ]; then
        echo -e "${GREEN}✓ $node found${NC}"
    else
        echo -e "${RED}✗ $node NOT found${NC}"
    fi
done

# 6. 테스트 스크립트 실행 준비
echo -e "\n${YELLOW}[5] Preparing test scripts...${NC}"
cd ~/ros2_cfi_project/cfi_lib

# 정상 동작 테스트 스크립트 실행 가능하게 설정
chmod +x ~/ros2_cfi_project/evaluation/day6_normal_operation_test.sh

echo -e "\n${GREEN}✅ Build complete! Ready to run tests.${NC}"
echo -e "\n${BLUE}To run normal operation tests:${NC}"
echo "cd ~/ros2_cfi_project/evaluation"
echo "./day6_normal_operation_test.sh"

echo -e "\n${BLUE}To test individual new nodes:${NC}"
echo "# Terminal 1 - Multi-topic node:"
echo "export LD_PRELOAD=~/ros2_cfi_project/cfi_lib/libday6_cfi.so"
echo "$BIN_DIR/multi_topic_node"

echo -e "\n# Terminal 2 - Service test node:"
echo "export LD_PRELOAD=~/ros2_cfi_project/cfi_lib/libday6_cfi.so"
echo "$BIN_DIR/service_test_node"

echo -e "\n# Terminal 3 - Performance monitor:"
echo "export LD_PRELOAD=~/ros2_cfi_project/cfi_lib/libday6_cfi.so"
echo "$BIN_DIR/performance_monitor"
