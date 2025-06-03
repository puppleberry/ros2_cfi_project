#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <cstring>

class SimpleVulnerable : public rclcpp::Node {
public:
    SimpleVulnerable() : Node("simple_vulnerable") {
        buffer_ = new char[64];
        memset(buffer_, 'X', 64);
        
        sub_ = this->create_subscription<std_msgs::msg::String>(
            "/attack", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                RCLCPP_INFO(this->get_logger(), "Got: %s", msg->data.c_str());
                
                if (msg->data.find("OVERFLOW:") == 0) {
                    // Buffer overflow
                    strcpy(buffer_, msg->data.c_str() + 9);
                    RCLCPP_WARN(this->get_logger(), "Overflow done!");
                }
            });
            
        RCLCPP_INFO(this->get_logger(), "Ready on /attack");
    }
    
    ~SimpleVulnerable() { delete[] buffer_; }
    
private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    char* buffer_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleVulnerable>());
    rclcpp::shutdown();
    return 0;
}
