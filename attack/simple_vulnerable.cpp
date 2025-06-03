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
