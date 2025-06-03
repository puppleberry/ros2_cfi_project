# Makefile for ROS2 CFI Attack/Defense Demo

.PHONY: all clean cfi test help package

all: cfi package

# CFI 라이브러리 빌드
cfi:
	@echo "Building CFI library..."
	@cd ../cfi_lib && \
	g++ -std=c++17 -fPIC -shared -o libday6_final_always_active.so day6_final_cfi_always_active.cpp -ldl -pthread -O2
	@echo "✅ CFI library built"

# ROS2 패키지 방식으로 빌드
package:
	@echo "Building vulnerable node using ROS2 package..."
	@bash -c 'source /opt/ros/humble/setup.bash && \
	cd ~/ros2_ws/src && \
	if [ ! -d "vulnerable_demo" ]; then \
		ros2 pkg create --build-type ament_cmake vulnerable_demo --dependencies rclcpp std_msgs; \
	fi && \
	cp ~/ros2_cfi_project/attack/simple_vulnerable.cpp ~/ros2_ws/src/vulnerable_demo/src/vulnerable_node.cpp && \
	cd ~/ros2_ws && \
	colcon build --packages-select vulnerable_demo --cmake-args -DCMAKE_CXX_FLAGS="-fno-stack-protector -O0" && \
	cp install/vulnerable_demo/lib/vulnerable_demo/vulnerable_node ~/ros2_cfi_project/attack/vulnerable_subscription'
	@chmod +x vulnerable_subscription 2>/dev/null || true
	@echo "✅ Vulnerable node built"

# 간단한 취약 노드 소스 생성
simple_vulnerable.cpp:
	@cat > simple_vulnerable.cpp << 'EOF'
	#include <memory>
	#include <cstring>
	#include "rclcpp/rclcpp.hpp"
	#include "std_msgs/msg/string.hpp"
	
	class VulnerableNode : public rclcpp::Node {
	private:
	  char buffer_[64];
	  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
	  
	public:
	  VulnerableNode() : Node("vulnerable_node") {
	    subscription_ = this->create_subscription<std_msgs::msg::String>(
	      "vulnerable_topic", 10,
	      [this](std_msgs::msg::String::SharedPtr msg) {
	        RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg->data.c_str());
	        
	        if (msg->data.find("OVERFLOW:") == 0) {
	          strcpy(buffer_, msg->data.c_str() + 9);
	          
	          if (msg->data.find("CORRUPT") != std::string::npos) {
	            RCLCPP_ERROR(this->get_logger(), "Corrupting subscription...");
	            unsigned char* ptr = (unsigned char*)subscription_.get();
	            for (int i = 100; i < 110; i++) ptr[i] ^= 0xFF;
	          }
	        }
	      });
	    RCLCPP_INFO(this->get_logger(), "Ready (PID: %d)", getpid());
	  }
	};
	
	int main(int argc, char* argv[]) {
	  rclcpp::init(argc, argv);
	  rclcpp::spin(std::make_shared<VulnerableNode>());
	  rclcpp::shutdown();
	  return 0;
	}
	EOF

# 테스트
test: all
	@echo "=== CFI Test ==="
	@echo "1. Starting vulnerable node with CFI..."
	@bash -c 'source /opt/ros/humble/setup.bash && \
	LD_PRELOAD=../cfi_lib/libday6_final_always_active.so ./vulnerable_subscription & \
	PID=$!; \
	sleep 2; \
	echo "2. Sending attack message..."; \
	ros2 topic pub --once /vulnerable_topic std_msgs/msg/String "data: \"OVERFLOW:CORRUPT\"" 2>/dev/null; \
	sleep 2; \
	kill $PID 2>/dev/null'

# 정리
clean:
	@rm -f vulnerable_subscription simple_vulnerable.cpp
	@rm -rf ~/ros2_ws/src/vulnerable_demo
	@echo "✅ Clean complete"

# 도움말
help:
	@echo "Simple ROS2 CFI Demo"
	@echo "==================="
	@echo "make all    - Build everything" 
	@echo "make test   - Run CFI test"
	@echo "make clean  - Clean files"
