#!/bin/bash
# test_robust_hooking.sh

echo "=== Robust Hooking Test ==="

# 1. 진단 도구 실행
echo -e "\n[1] Running diagnostic..."
g++ -fPIC -shared -o libdiag.so diagnostic_tool.cpp -ldl
LD_PRELOAD=./libdiag.so ros2 run basic_communication subscriber &
DIAG_PID=$!
sleep 2
kill $DIAG_PID 2>/dev/null

# 2. 강력한 후킹 테스트
echo -e "\n[2] Testing robust hooking..."
g++ -fPIC -shared -o librobust.so robust_hooking.cpp -ldl

# 직접 바이너리 실행
BINARY=$(find ~/ros2_ws -name "subscriber" -type f -executable | grep -v ".py" | head -1)
if [ -n "$BINARY" ]; then
    echo "Testing with binary: $BINARY"
    LD_PRELOAD=./librobust.so $BINARY &
    SUB_PID=$!
    sleep 2
    
    # 메시지 전송
    ros2 topic pub --once /topic std_msgs/msg/String "data: 'Hook test'" 
    
    sleep 1
    kill $SUB_PID 2>/dev/null
fi
