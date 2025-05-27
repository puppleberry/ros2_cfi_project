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
