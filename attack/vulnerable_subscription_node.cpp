// vulnerable_node.cpp - 취약한 ROS2 노드
#include <memory>
#include <cstring>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class VulnerableSubscriber : public rclcpp::Node {
public:
  VulnerableSubscriber() : Node("vulnerable_subscriber") {
    // 버퍼 초기화
    memset(buffer_, 0, sizeof(buffer_));
    
    // Subscription 생성
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&VulnerableSubscriber::topic_callback, this, _1));
      
    RCLCPP_INFO(this->get_logger(), "=== Vulnerable Node Started ===");
    RCLCPP_INFO(this->get_logger(), "PID: %d", getpid());
    RCLCPP_INFO(this->get_logger(), "Buffer at: %p", buffer_);
    RCLCPP_INFO(this->get_logger(), "Subscription SharedPtr at: %p", &subscription_);
    RCLCPP_INFO(this->get_logger(), "Subscription object at: %p", subscription_.get());
    
    // SharedPtr 내부 구조 출력
    void** ptr_array = (void**)&subscription_;
    RCLCPP_INFO(this->get_logger(), "SharedPtr internal pointers:");
    for (int i = 0; i < 2; i++) {
      RCLCPP_INFO(this->get_logger(), "  [%d]: %p", i, ptr_array[i]);
    }
  }

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) {
    callback_count_++;
    RCLCPP_INFO(this->get_logger(), "[Callback #%d] Received: '%s'", 
                callback_count_, msg->data.c_str());
    
    // 취약점: Buffer overflow
    if (msg->data.find("OVERFLOW:") == 0) {
      const char* payload = msg->data.c_str() + 9;  // "OVERFLOW:" 이후
      
      RCLCPP_WARN(this->get_logger(), "⚠️  Processing overflow message...");
      strcpy(buffer_, payload);  // 위험: 길이 체크 없음!
      
      // Subscription 변조 시도
      if (msg->data.find("CORRUPT") != std::string::npos && subscription_) {
        RCLCPP_ERROR(this->get_logger(), "🔥 Attempting to corrupt subscription...");
        
        // SharedPtr이 가리키는 실제 객체를 변조
        void* actual_sub = subscription_.get();
        RCLCPP_ERROR(this->get_logger(), "Target subscription object at: %p", actual_sub);
        
        // SharedPtr 자체의 주소도 출력
        RCLCPP_ERROR(this->get_logger(), "SharedPtr address: %p", &subscription_);
        void** ptr_array = (void**)&subscription_;
        RCLCPP_ERROR(this->get_logger(), "SharedPtr[0] (object): %p", ptr_array[0]);
        RCLCPP_ERROR(this->get_logger(), "SharedPtr[1] (control): %p", ptr_array[1]);
        
        unsigned char* ptr = (unsigned char*)actual_sub;
        
        // 변조 전 바이트 출력
        RCLCPP_ERROR(this->get_logger(), "Bytes 100-109 BEFORE corruption:");
        for (int i = 100; i < 110; i++) {
          RCLCPP_ERROR(this->get_logger(), "  [%d]: 0x%02X", i, ptr[i]);
        }
        
        // CFI가 체크하는 첫 256바이트 중 일부를 변경
        for (int i = 100; i < 110; i++) {
          ptr[i] ^= 0xFF;  // XOR로 변조
        }
        
        // 변조 후 바이트 출력
        RCLCPP_ERROR(this->get_logger(), "Bytes 100-109 AFTER corruption:");
        for (int i = 100; i < 110; i++) {
          RCLCPP_ERROR(this->get_logger(), "  [%d]: 0x%02X", i, ptr[i]);
        }
        
        RCLCPP_ERROR(this->get_logger(), "✅ Subscription memory corrupted!");
        RCLCPP_ERROR(this->get_logger(), "Next callback should trigger CFI violation...");
      }
    } else {
      // 정상 메시지는 안전하게 처리
      strncpy(buffer_, msg->data.c_str(), sizeof(buffer_) - 1);
    }
  }
  
  char buffer_[64];  // 취약한 버퍼
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  int callback_count_ = 0;
};

int main(int argc, char * argv[]) {
  printf("=== Vulnerable Subscription Node ===\n");
  printf("This node has a buffer overflow vulnerability.\n");
  printf("It was compiled without stack protector.\n\n");
  
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VulnerableSubscriber>());
  rclcpp::shutdown();
  
  return 0;
}
