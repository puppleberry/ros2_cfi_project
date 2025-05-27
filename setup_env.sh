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
