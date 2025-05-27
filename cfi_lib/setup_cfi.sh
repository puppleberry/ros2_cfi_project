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

# 현재 디렉토리가 이미 cfi_lib인지 확인
CURRENT_DIR=$(pwd)
if [[ "$CURRENT_DIR" == *"cfi_lib"* ]]; then
    echo "Already in cfi_lib directory, skipping file copy"
else
    cp ros2_cfi_lib.cpp $CFI_LIB_DIR/
    cp test_cfi.cpp $CFI_LIB_DIR/
    cp Makefile $CFI_LIB_DIR/
fi

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

# 7. 추가 스크립트 파일들 생성
echo -e "\n${YELLOW}Creating additional scripts...${NC}"

# test_hook.sh 생성
cat > $CFI_LIB_DIR/test_hook.sh << 'EOFSCRIPT'
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
EOFSCRIPT

chmod +x $CFI_LIB_DIR/test_hook.sh

# analyze_subscription.sh 생성
cat > $CFI_LIB_DIR/analyze_subscription.sh << 'EOFSCRIPT'
#!/bin/bash
# analyze_subscription.sh - ROS2 Subscription 클래스 구조 분석

echo "=== Analyzing ROS2 Subscription Structure ==="

# 1. SubscriptionBase 관련 함수들 찾기
echo -e "\n[1] SubscriptionBase virtual functions:"
nm -D /opt/ros/humble/lib/librclcpp.so | grep -i "subscriptionbase" | grep -E "handle|execute|callback" | head -20

# 2. handle_message 관련 함수들
echo -e "\n[2] handle_message functions:"
nm -D /opt/ros/humble/lib/librclcpp.so | grep -i "handle_message" | head -10

# 3. execute 관련 함수들
echo -e "\n[3] execute functions:"
nm -D /opt/ros/humble/lib/librclcpp.so | grep -i "subscription.*execute" | head -10

# 4. AnySubscriptionCallback 관련 (실제 콜백이 저장되는 곳)
echo -e "\n[4] AnySubscriptionCallback:"
nm -D /opt/ros/humble/lib/librclcpp.so | grep -i "anysubscriptioncallback" | head -10

# 5. 실제 메시지 디스패치 함수들
echo -e "\n[5] Message dispatch functions:"
nm -D /opt/ros/humble/lib/librclcpp.so | grep -E "dispatch|invoke.*callback" | head -10

# 6. std_msgs String 관련 subscription
echo -e "\n[6] String message subscriptions:"
nm -D /opt/ros/humble/lib/librclcpp.so | grep "SubscriptionI.*String" | head -10

# 7. 가장 유망한 후킹 타겟 찾기
echo -e "\n[7] Most promising hooking targets:"
echo "Looking for functions that are called when messages arrive..."
nm -D /opt/ros/humble/lib/librclcpp.so | grep -E "_ZN.*Subscription.*handle_message|_ZN.*Subscription.*execute_impl|_ZN.*AnySubscriptionCallback.*dispatch" | head -20

# 8. 심볼 demangle
echo -e "\n[8] Demangling some key symbols:"
echo "_ZN6rclcpp16SubscriptionBase13handle_messageESt10shared_ptrIN3rcl3msg14MessageInfoPtrEEOS2_IvE" | c++filt
echo "_ZN6rclcpp16SubscriptionBase12execute_implESt10shared_ptrIvE" | c++filt

# 9. RTTI 정보 확인 (타입 정보)
echo -e "\n[9] RTTI Information:"
nm -D /opt/ros/humble/lib/librclcpp.so | grep -i "typeinfo.*subscription" | head -10
EOFSCRIPT

chmod +x $CFI_LIB_DIR/analyze_subscription.sh

# 8. 결과 출력
echo -e "\n${GREEN}=== Setup Complete ===${NC}"
echo -e "Project directory: ${YELLOW}$PROJECT_ROOT${NC}"
echo -e "\nNext steps:"
echo -e "1. ${YELLOW}cd $PROJECT_ROOT${NC}"
echo -e "2. ${YELLOW}source setup_env.sh${NC}"
echo -e "3. Start implementing ROS2 function hooks in ${YELLOW}cfi_lib/ros2_cfi_lib.cpp${NC}"
echo -e "\nCheck the log file: ${YELLOW}/tmp/ros2_cfi.log${NC}"
