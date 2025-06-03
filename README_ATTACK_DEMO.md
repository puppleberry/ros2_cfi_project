# ROS2 CFI Attack/Defense Demonstration

## 개요
이 데모는 ROS2 subscription 콜백에 대한 제어 흐름 탈취 공격과 CFI(Control Flow Integrity)를 통한 방어를 시연합니다.

## 핵심 구성요소

### 1. 취약한 노드 (`vulnerable_subscription_node.cpp`)
- Buffer overflow 취약점이 의도적으로 포함된 ROS2 노드
- 보호 기능 없이 컴파일 (`-fno-stack-protector -z execstack -no-pie`)
- Subscription 객체 메모리 변조 가능

### 2. CFI 라이브러리 (`day6_final_cfi_always_active.cpp`)
- `execute_subscription()` 함수 후킹
- Subscription 객체의 256바이트 해시 계산 및 검증
- 변조 감지 시 프로세스 종료

### 3. 공격 스크립트 (`attack_demo.py`)
- 단계별 공격 시퀀스 실행
- Buffer overflow → Subscription 변조 → CFI 트리거

## 빠른 시작

### 1. 프로젝트 설정
```bash
cd ~/ros2_cfi_project
mkdir -p attack cfi_lib
```

### 2. 파일 배치
- `attack/` 디렉토리에:
  - `vulnerable_subscription_node.cpp`
  - `attack_demo.py`
  - `run_cfi_demo.sh`
  - `Makefile`
- `cfi_lib/` 디렉토리에:
  - `day6_final_cfi_always_active.cpp`

### 3. 빌드
```bash
cd attack
make all
```

### 4. 데모 실행
```bash
make demo
# 또는
./run_cfi_demo.sh
```

## 시연 시나리오

### Scenario 1: CFI 없이 공격 (취약)
```bash
# 취약한 노드 실행
./vulnerable_subscription

# 다른 터미널에서 공격 실행
python3 attack_demo.py
```
**결과**: Subscription 변조가 감지되지 않음

### Scenario 2: CFI 적용 공격 (보호됨)
```bash
# CFI로 보호된 노드 실행
LD_PRELOAD=../cfi_lib/libday6_final_always_active.so ./vulnerable_subscription

# 다른 터미널에서 공격 실행
python3 attack_demo.py
```
**결과**: "🚨 CFI VIOLATION DETECTED!" → 프로세스 종료

## 공격 메커니즘

1. **정상 메시지**: 정상 동작 확인
2. **Buffer Overflow**: 취약한 strcpy()로 메모리 오염
3. **Subscription 변조**: `OVERFLOW:CORRUPT_SUB` 메시지로 subscription 객체 변경
4. **CFI 트리거**: 다음 콜백에서 해시 불일치 감지

## 주요 로그 메시지

### CFI 활성화
```
[CFI-SUCCESS] === CFI Protection Active! First callback intercepted ===
[CFI-SUCCESS] Registered new subscription 0x... (hash: 0x...)
```

### 공격 감지
```
[CFI-ERROR] 🚨 CFI VIOLATION DETECTED! 🚨
[CFI-ERROR]   Expected hash: 0x7e4492b18e505ef8
[CFI-ERROR]   Current hash: 0x6d3b82a17a505ef8
[CFI-ERROR]   ❌ BLOCKING EXECUTION (strict mode)
[CFI-ERROR] 🛑 Execution blocked by CFI!
```

## 문제 해결

### CFI가 활성화되지 않는 경우
1. `is_ros2_node()` 함수가 true를 반환하는지 확인
2. `libday6_final_always_active.so` 사용 (항상 활성화 버전)
3. 환경 변수 확인: `ROS2_CFI_DEBUG=1`

### Segfault가 먼저 발생하는 경우
1. 컴파일 플래그 확인 (`-fno-stack-protector` 필수)
2. Overflow 크기 조정 (너무 크면 즉시 크래시)
3. Subscription 변조 방식을 XOR로 변경 (직접 덮어쓰기 대신)

### 변조가 감지되지 않는 경우
1. CFI가 실제로 로드되었는지 확인 (첫 콜백 메시지)
2. Subscription 주소가 올바른지 확인
3. 해시 계산 범위(256바이트) 내에서 변조가 일어나는지 확인

## 성능 측정

CFI 오버헤드 측정:
```bash
# Performance monitor 사용
LD_PRELOAD=../cfi_lib/libday6_final.so \
~/ros2_ws/install/basic_communication/lib/basic_communication/performance_monitor
```

## 핵심 성과

1. **작동하는 공격**: Buffer overflow로 실제 subscription 변조
2. **효과적인 방어**: CFI가 변조를 감지하고 차단
3. **낮은 오버헤드**: 평균 7.8μs 추가 레이턴시
4. **실용적 구현**: LD_PRELOAD로 즉시 적용 가능

## 다음 단계

1. 다양한 공격 패턴 추가
2. CFI 우회 시도 및 방어
3. 실제 로봇 시스템 적용
4. 성능 최적화 (해시 테이블 개선)
