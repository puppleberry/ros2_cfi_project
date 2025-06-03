#!/bin/bash
# setup_attack_demo.sh - 공격 데모 환경 자동 구성

set -e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

PROJECT_DIR="$HOME/ros2_cfi_project"

echo -e "${YELLOW}Setting up ROS2 CFI Attack Demo Environment...${NC}"

# 1. 디렉토리 구조 생성
echo "Creating directory structure..."
mkdir -p $PROJECT_DIR/{vulnerable_node,exploit,evaluation/attack_logs}

# 2. 파일 복사 (이미 artifacts로 생성된 파일들)
echo "Note: Copy the following files to their locations:"
echo "  - vulnerable_target.cpp → $PROJECT_DIR/vulnerable_node/"
echo "  - attack_demo.py → $PROJECT_DIR/exploit/"
echo "  - generate_payload.py → $PROJECT_DIR/exploit/"
echo "  - test_attack_defense.sh → $PROJECT_DIR/"
echo "  - quick_demo.sh → $PROJECT_DIR/"

# 3. 실행 권한 설정
cd $PROJECT_DIR
chmod +x test_attack_defense.sh quick_demo.sh 2>/dev/null || true
chmod +x exploit/*.py 2>/dev/null || true

# 4. CFI 라이브러리 확인
echo -e "\n${YELLOW}Checking CFI library...${NC}"
if [ -f "$PROJECT_DIR/cfi_lib/libday6_final.so" ]; then
    echo -e "${GREEN}✓ CFI library found${NC}"
else
    echo "⚠ CFI library not found. Please compile it first:"
    echo "  cd $PROJECT_DIR/cfi_lib"
    echo "  g++ -std=c++17 -fPIC -shared -o libday6_final.so day6_final_cfi.cpp -ldl -pthread -O2"
fi

# 5. 사용 방법 출력
echo -e "\n${GREEN}=== Setup Complete ===${NC}"
echo ""
echo "Quick start:"
echo "1. Build vulnerable node:"
echo "   cd ~/ros2_ws && colcon build --packages-select vulnerable_node"
echo ""
echo "2. Run automated test:"
echo "   cd $PROJECT_DIR && ./test_attack_defense.sh"
echo ""
echo "3. Or manual demo:"
echo "   ./quick_demo.sh"
echo ""
echo "Files location:"
echo "  - Attack logs: $PROJECT_DIR/evaluation/attack_logs/"
echo "  - CFI library: $PROJECT_DIR/cfi_lib/libday6_final.so"

echo -e "\n${GREEN}Ready for attack demonstration!${NC}"
