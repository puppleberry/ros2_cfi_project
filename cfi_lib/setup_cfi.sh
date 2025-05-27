#!/bin/bash
# setup_cfi.sh - ROS2 CFI 프로젝트 설정 및 빌드 스크립트

# 색상 정의
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 프로젝트 루트 디렉토리
PROJECT_ROOT=~/ros2_cfi_project
CFI_LIB_DIR=$PROJECT_ROOT/cfi_lib

echo -e "${GREEN}=== ROS2 CFI Project Setup ===${NC}"

# 1. 디렉토리 구조 생성
echo -e "${YELLOW}Creating project directories...${NC}"
mkdir -p $PROJECT_ROOT/{cfi_lib,exploit,evaluation,docs,logs}

# 2. 파일 복사 (현재 디렉토리에 있다고 가정)
echo -e "${YELLOW}Copying files...${NC}"
cp ros2_cfi_lib.cpp $CFI_LIB_DIR/
cp test_cfi.cpp $CFI_LIB_DIR/
cp Makefile $CFI_LIB_DIR/

# 3. CFI 라이브러리 빌드
echo -e "${YELLOW}Building CFI library...${NC}"
cd $CFI_LIB_DIR
make clean
make all

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Build successful!${NC}"
else
    echo -e "${RED}✗ Build failed!${NC}"
    exit 1
fi

# 4. 기본 테스트 실행
echo -e "\n${YELLOW}Running basic test...${NC}"
make test

# 5. 환경 설정 파일 생성
echo -e "\n${YELLOW}Creating environment setup file...${NC}"
cat > $PROJECT_ROOT/setup_env.sh << 'EOF'
#!/bin/bash
# ROS2 CFI 환경 설정

# CFI 라이브러리 경로
export CFI_LIB_PATH=~/ros2_cfi_project/cfi_lib/libros2_cfi.so

# 로그 레벨 설정 (DEBUG, INFO, WARN, ERROR)
export ROS2_CFI_LOG_LEVEL=INFO

# CFI 활성화 (0=활성화, 1=비활성화)
export ROS2_CFI_DISABLE=0

# 함수: CFI와 함께 ROS2 노드 실행
run_with_cfi() {
    if [ $# -lt 2 ]; then
        echo "Usage: run_with_cfi <package> <node>"
        return 1
    fi
    
    echo "Running $2 from $1 with CFI protection..."
    LD_PRELOAD=$CFI_LIB_PATH ros2 run $1 $2
}

# 함수: CFI 로그 확인
cfi_log() {
    tail -f /tmp/ros2_cfi.log
}

# 함수: CFI 로그 정리
cfi_clean_log() {
    rm -f /tmp/ros2_cfi.log
    echo "CFI log cleaned"
}

echo "ROS2 CFI environment loaded"
echo "Available commands:"
echo "  run_with_cfi <package> <node> - Run ROS2 node with CFI"
echo "  cfi_log                       - View CFI logs"
echo "  cfi_clean_log                 - Clean CFI logs"
EOF

chmod +x $PROJECT_ROOT/setup_env.sh

# 6. README 생성
echo -e "\n${YELLOW}Creating README...${NC}"
cat > $PROJECT_ROOT/README.md << 'EOF'
# ROS2 CFI (Control Flow Integrity) Project

## 프로젝트 구조
```
ros2_cfi_project/
├── cfi_lib/           # CFI 라이브러리
├── exploit/           # 공격 코드
├── evaluation/        # 평가 스크립트
├── docs/              # 문서
└── logs/              # 로그 파일
```

## 빌드 방법
```bash
cd cfi_lib
make all
```

## 사용 방법

### 1. 환경 설정
```bash
source ~/ros2_cfi_project/setup_env.sh
```

### 2. ROS2 노드와 함께 실행
```bash
# 방법 1: run_with_cfi 함수 사용
run_with_cfi basic_communication vulnerable_subscriber

# 방법 2: 직접 LD_PRELOAD 사용
LD_PRELOAD=~/ros2_cfi_project/cfi_lib/libros2_cfi.so ros2 run basic_communication vulnerable_subscriber
```

### 3. 로그 확인
```bash
cfi_log  # 또는 tail -f /tmp/ros2_cfi.log
```

## 환경 변수
- `ROS2_CFI_DISABLE`: CFI 비활성화 (1로 설정 시)
- `ROS2_CFI_LOG_LEVEL`: 로그 레벨 (DEBUG/INFO/WARN/ERROR)

## 다음 단계
- [ ] rclcpp subscription 함수 후킹
- [ ] 콜백 화이트리스트 구현
- [ ] 실제 공격 방어 테스트
EOF

# 7. 결과 출력
echo -e "\n${GREEN}=== Setup Complete ===${NC}"
echo -e "Project directory: ${YELLOW}$PROJECT_ROOT${NC}"
echo -e "\nNext steps:"
echo -e "1. ${YELLOW}cd $PROJECT_ROOT${NC}"
echo -e "2. ${YELLOW}source setup_env.sh${NC}"
echo -e "3. Start implementing ROS2 function hooks in ${YELLOW}cfi_lib/ros2_cfi_lib.cpp${NC}"
echo -e "\nCheck the log file: ${YELLOW}/tmp/ros2_cfi.log${NC}"
