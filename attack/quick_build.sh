#!/bin/bash
# quick_build.sh - ROS2 환경 변수를 활용한 빠른 빌드 스크립트

set -e

echo "Setting up ROS2 environment..."
source /opt/ros/humble/setup.bash

echo "Building CFI libraries..."
cd ~/ros2_cfi_project/cfi_lib

# CFI 라이브러리 컴파일
g++ -std=c++17 -fPIC -shared -o libday6_final.so day6_final_cfi.cpp -ldl -pthread -O2
g++ -std=c++17 -fPIC -shared -o libday6_final_always_active.so day6_final_cfi_always_active.cpp -ldl -pthread -O2

echo "✅ CFI libraries built"

echo "Building vulnerable node..."
cd ~/ros2_cfi_project/attack

# ROS2 패키지의 모든 include 디렉토리를 자동으로 포함
INCLUDES=""
for dir in /opt/ros/humble/include/*; do
    if [ -d "$dir" ]; then
        INCLUDES="$INCLUDES -I$dir"
    fi
done

# 취약한 노드 컴파일
g++ -o vulnerable_subscription vulnerable_subscription_node.cpp \
    $INCLUDES \
    -L/opt/ros/humble/lib \
    -Wl,-rpath,/opt/ros/humble/lib \
    -lrclcpp -lrcl -lrcl_interfaces__rosidl_typesupport_cpp \
    -lrcl_yaml_param_parser -lrmw -lrmw_implementation \
    -lrcutils -lrcpputils -lrosidl_typesupport_cpp \
    -lrosidl_runtime_cpp -lstd_msgs__rosidl_typesupport_cpp \
    -lbuiltin_interfaces__rosidl_typesupport_cpp \
    -llibstatistics_collector -ltracetools \
    -fno-stack-protector -z execstack -no-pie -Wl,-z,norelro -O0

echo "✅ Vulnerable node built"

# 실행 권한 부여
chmod +x vulnerable_subscription
chmod +x attack_demo.py
chmod +x run_cfi_demo.sh

echo ""
echo "Build complete! You can now run:"
echo "  ./run_cfi_demo.sh"
echo "or"
echo "  make demo"
