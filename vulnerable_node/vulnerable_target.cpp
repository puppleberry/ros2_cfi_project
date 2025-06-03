// vulnerable_target.cpp
// ROS2 subscription 메모리 corruption 취약점을 가진 노드

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <cstring>
#include <csignal>

class VulnerableNode : public rclcpp::Node
{
public:
    VulnerableNode() : Node("vulnerable_target")
    {
        RCLCPP_INFO(this->get_logger(), "Vulnerable node started!");
        
        // 정상 subscription
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/attack_topic", 10,
            std::bind(&VulnerableNode::topic_callback, this, std::placeholders::_1));
            
        // 취약한 버퍼 (subscription 객체 바로 뒤에 위치하도록)
        vulnerable_buffer_ = new char[64];
        memset(vulnerable_buffer_, 0, 64);
        
        RCLCPP_INFO(this->get_logger(), "Subscription ptr: %p", subscription_.get());
        RCLCPP_INFO(this->get_logger(), "Buffer ptr: %p", vulnerable_buffer_);
        RCLCPP_INFO(this->get_logger(), "Node ready to receive messages on /attack_topic");
    }
    
    ~VulnerableNode() {
        delete[] vulnerable_buffer_;
    }

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received: '%s' (len=%zu)", 
                    msg->data.c_str(), msg->data.length());
        
        // 취약점: 길이 검사 없이 복사
        if (msg->data.find("OVERFLOW:") == 0) {
            // OVERFLOW: 프리픽스가 있으면 취약한 복사 수행
            const char* payload = msg->data.c_str() + 9; // "OVERFLOW:" 이후
            
            RCLCPP_WARN(this->get_logger(), "Processing overflow data...");
            
            // 의도적으로 취약한 strcpy 사용
            strcpy(vulnerable_buffer_, payload);
            
            // Subscription 근처 메모리 상태 출력
            unsigned char* sub_mem = reinterpret_cast<unsigned char*>(subscription_.get());
            RCLCPP_INFO(this->get_logger(), "Subscription memory after copy:");
            for (int i = 0; i < 32; i += 8) {
                RCLCPP_INFO(this->get_logger(), "  +%02d: %02x %02x %02x %02x %02x %02x %02x %02x",
                    i, sub_mem[i], sub_mem[i+1], sub_mem[i+2], sub_mem[i+3],
                    sub_mem[i+4], sub_mem[i+5], sub_mem[i+6], sub_mem[i+7]);
            }
        } else if (msg->data == "TRIGGER") {
            // 다음 콜백 호출을 트리거 (CFI 검증 유발)
            RCLCPP_INFO(this->get_logger(), "Triggering next callback...");
            // 다음 메시지를 기다림
        } else {
            // 정상 처리
            RCLCPP_INFO(this->get_logger(), "Normal processing: %s", msg->data.c_str());
        }
    }
    
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    char* vulnerable_buffer_;
};

// 시그널 핸들러
void signal_handler(int sig) {
    if (sig == SIGSEGV) {
        std::cerr << "\n[ATTACK SUCCESS] Segmentation fault - memory corrupted!\n";
        exit(1);
    } else if (sig == SIGABRT) {
        std::cerr << "\n[CFI DEFENSE] Program aborted by CFI!\n";
        exit(2);
    }
}

int main(int argc, char* argv[])
{
    // 시그널 핸들러 설정
    signal(SIGSEGV, signal_handler);
    signal(SIGABRT, signal_handler);
    
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<VulnerableNode>();
    
    std::cout << "\n=== Vulnerable Target Node ===\n";
    std::cout << "This node has a buffer overflow vulnerability.\n";
    std::cout << "Send messages to /attack_topic to test.\n";
    std::cout << "Use 'OVERFLOW:' prefix to trigger vulnerable code path.\n\n";
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}
