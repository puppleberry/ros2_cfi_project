#!/bin/bash
# test_improved_cfi.sh - 개선된 CFI 테스트

set -e

echo "=== Improved CFI Test ==="

cd ~/ros2_cfi_project/cfi_lib

# 1. 빌드
echo "Building improved CFI..."
g++ -std=c++17 -fPIC -shared -o libimproved_cfi.so improved_cfi.cpp -ldl

# 2. 환경 설정
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# 3. 간단한 테스트
echo -e "\n>>> Quick Test with normal subscriber"
timeout 8s bash -c "
cd ~/ros2_ws
LD_PRELOAD=~/ros2_cfi_project/cfi_lib/libimproved_cfi.so ros2 run basic_communication subscriber &
SUB_PID=\$!
sleep 3
ros2 topic pub --times 3 --rate 1 /topic std_msgs/msg/String 'data: \"Test\"' &
wait \$SUB_PID
"

echo -e "\nCFI test complete. Check the output for violations."
