#!/bin/bash
# quick_test_cfi.sh - 빠른 CFI 테스트

set -e

echo "=== Quick CFI Test ==="

cd ~/ros2_cfi_project/cfi_lib

# 1. 빌드
echo "Building simple attack detector..."
g++ -std=c++17 -fPIC -shared -o libsimple_detector.so simple_attack_detector.cpp -ldl

# 2. 환경 설정
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# 3. 정상 동작 테스트
echo -e "\n>>> Normal Operation Test"
echo "Starting subscriber..."

# Terminal 1 명령
echo -e "\nRun this in Terminal 1:"
echo "cd ~/ros2_ws"
echo "LD_PRELOAD=~/ros2_cfi_project/cfi_lib/libsimple_detector.so ros2 run basic_communication subscriber"

echo -e "\nThen run this in Terminal 2:"
echo "ros2 topic pub /topic std_msgs/msg/String \"data: 'Hello World'\""

echo -e "\n>>> Vulnerable Node Test (if available)"
echo -e "\nFor vulnerable node test, run in Terminal 1:"
echo "cd ~/ros2_ws"
echo "LD_PRELOAD=~/ros2_cfi_project/cfi_lib/libsimple_detector.so ROS2_CFI_STRICT=0 ros2 run basic_communication vulnerable_subscriber"

echo -e "\nThen in Terminal 2:"
echo "# Normal message:"
echo "ros2 topic pub --once /vulnerable_topic std_msgs/msg/String \"data: 'Hello'\""
echo "# Attack message (buffer overflow):"
echo "ros2 topic pub --once /vulnerable_topic std_msgs/msg/String \"data: 'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA'\""

echo -e "\nLibrary built successfully: libsimple_detector.so"
