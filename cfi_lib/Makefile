# Makefile for ROS2 CFI Library

CXX = g++
CXXFLAGS = -std=c++17 -fPIC -Wall -Wextra -O2 -g
LDFLAGS = -shared -ldl -pthread

# ROS2 관련 경로 (필요시 수정)
ROS2_INCLUDE = /opt/ros/humble/include
ROS2_LIB = /opt/ros/humble/lib

# 타겟 파일
TARGET = libros2_cfi.so
SOURCES = ros2_cfi_lib.cpp
OBJECTS = $(SOURCES:.cpp=.o)

# 테스트 프로그램
TEST_TARGET = cfi_test
TEST_SOURCES = test_cfi.cpp

# 기본 타겟
all: $(TARGET) $(TEST_TARGET)

# 공유 라이브러리 빌드
$(TARGET): $(OBJECTS)
	$(CXX) $(LDFLAGS) -o $@ $^
	@echo "✓ CFI library built: $(TARGET)"

# 오브젝트 파일 컴파일
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -I$(ROS2_INCLUDE) -c $< -o $@

# 테스트 프로그램 빌드
$(TEST_TARGET): $(TEST_SOURCES)
	$(CXX) $(CXXFLAGS) -o $@ $< -ldl
	@echo "✓ Test program built: $(TEST_TARGET)"

# CFI 라이브러리 테스트 실행
test: $(TARGET) $(TEST_TARGET)
	@echo "=== Running CFI Library Test ==="
	LD_PRELOAD=./$(TARGET) ./$(TEST_TARGET)
	@echo "=== Test Complete ==="
	@echo "Check log file: /tmp/ros2_cfi.log"

# 로그 파일 확인
log:
	@tail -f /tmp/ros2_cfi.log

# 로그 파일 삭제
clean-log:
	@rm -f /tmp/ros2_cfi.log
	@echo "✓ Log file cleaned"

# 빌드 정리
clean:
	rm -f $(OBJECTS) $(TARGET) $(TEST_TARGET)
	@echo "✓ Build files cleaned"

# 전체 정리 (로그 포함)
distclean: clean clean-log

# ROS2 노드와 함께 테스트 (vulnerable_subscriber 사용)
test-ros2: $(TARGET)
	@echo "=== Testing with ROS2 Node ==="
	@echo "Starting vulnerable_subscriber with CFI protection..."
	cd ~/ros2_ws && \
	source /opt/ros/humble/setup.bash && \
	colcon build --packages-select basic_communication && \
	source install/setup.bash && \
	LD_PRELOAD=$(PWD)/$(TARGET) ROS2_CFI_LOG_LEVEL=DEBUG ros2 run basic_communication vulnerable_subscriber

# 디버깅 모드로 실행
debug: $(TARGET)
	@echo "=== Debug Mode ==="
	LD_PRELOAD=./$(TARGET) ROS2_CFI_LOG_LEVEL=DEBUG gdb ./$(TEST_TARGET)

# 도움말
help:
	@echo "ROS2 CFI Library Makefile"
	@echo "========================"
	@echo "Targets:"
	@echo "  make all       - Build library and test program"
	@echo "  make test      - Run basic test"
	@echo "  make test-ros2 - Test with ROS2 vulnerable node"
	@echo "  make log       - View log file (tail -f)"
	@echo "  make clean     - Clean build files"
	@echo "  make distclean - Clean everything including logs"
	@echo "  make debug     - Run test in GDB with CFI"
	@echo ""
	@echo "Environment Variables:"
	@echo "  ROS2_CFI_DISABLE=1    - Disable CFI protection"
	@echo "  ROS2_CFI_LOG_LEVEL    - Set log level (DEBUG/INFO/WARN/ERROR)"
	@echo ""
	@echo "Usage Example:"
	@echo "  LD_PRELOAD=./libros2_cfi.so ros2 run your_package your_node"

.PHONY: all test test-ros2 log clean clean-log distclean debug help
