// final_working_attack.cpp
// CFI가 확실히 감지할 수 있는 공격 데모

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <cstring>
#include <vector>

class FinalAttackDemo : public rclcpp::Node
{
public:
    FinalAttackDemo() : Node("final_attack_demo"), corrupted_(false)
    {
        RCLCPP_INFO(this->get_logger(), "\n=== Final CFI Attack Demo ===");
        
        // Subscription 생성
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/attack_topic", 10,
            std::bind(&FinalAttackDemo::topic_callback, this, std::placeholders::_1));
        
        // Subscription raw pointer 저장
        sub_ptr_ = subscription_.get();
        
        // 백업용으로 원본 메모리 상태 저장
        backup_memory();
        
        RCLCPP_INFO(this->get_logger(), "Node initialized successfully");
        RCLCPP_INFO(this->get_logger(), "Subscription pointer: %p", sub_ptr_);
        RCLCPP_INFO(this->get_logger(), "\nCommands:");
        RCLCPP_INFO(this->get_logger(), "  'CHECK' - Check subscription state");
        RCLCPP_INFO(this->get_logger(), "  'CORRUPT' - Corrupt subscription memory");
        RCLCPP_INFO(this->get_logger(), "  'TEST' - Test callback (will trigger CFI if corrupted)");
        RCLCPP_INFO(this->get_logger(), "================================\n");
    }

private:
    void backup_memory()
    {
        // Subscription의 첫 64바이트를 백업
        unsigned char* mem = reinterpret_cast<unsigned char*>(sub_ptr_);
        original_bytes_.resize(64);
        std::memcpy(original_bytes_.data(), mem, 64);
    }
    
    void print_memory_state()
    {
        unsigned char* mem = reinterpret_cast<unsigned char*>(sub_ptr_);
        
        RCLCPP_INFO(this->get_logger(), "Subscription memory (first 32 bytes):");
        for (int i = 0; i < 32; i += 8) {
            RCLCPP_INFO(this->get_logger(), "  +%02d: %02x %02x %02x %02x %02x %02x %02x %02x",
                i, mem[i], mem[i+1], mem[i+2], mem[i+3], 
                mem[i+4], mem[i+5], mem[i+6], mem[i+7]);
        }
        
        // 변경된 바이트 수 계산
        int changed = 0;
        for (int i = 0; i < 64; i++) {
            if (mem[i] != original_bytes_[i]) changed++;
        }
        
        if (changed > 0) {
            RCLCPP_WARN(this->get_logger(), "%d bytes have been modified!", changed);
        }
    }
    
    void corrupt_subscription()
    {
        RCLCPP_ERROR(this->get_logger(), "\n!!! CORRUPTING SUBSCRIPTION MEMORY !!!");
        
        unsigned char* mem = reinterpret_cast<unsigned char*>(sub_ptr_);
        
        // 옵션 1: 첫 8바이트를 패턴으로 덮어쓰기
        // 이는 vtable 포인터를 손상시킬 가능성이 높음
        for (int i = 0; i < 8; i++) {
            mem[i] = 0x41 + i;  // 41, 42, 43, ... (A, B, C, ...)
        }
        
        // 옵션 2: 더 많은 영역 손상 (더 확실한 감지)
        // for (int i = 8; i < 16; i++) {
        //     mem[i] = 0xFF;
        // }
        
        corrupted_ = true;
        
        RCLCPP_ERROR(this->get_logger(), "Subscription memory has been corrupted!");
        print_memory_state();
        RCLCPP_ERROR(this->get_logger(), "\nNext callback should trigger CFI violation!");
    }
    
    void topic_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        static int callback_count = 0;
        callback_count++;
        
        RCLCPP_INFO(this->get_logger(), "\n--- Callback #%d ---", callback_count);
        RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg->data.c_str());
        
        if (msg->data == "CHECK") {
            RCLCPP_INFO(this->get_logger(), "Checking subscription state...");
            print_memory_state();
            
            if (corrupted_) {
                RCLCPP_WARN(this->get_logger(), "Subscription is CORRUPTED!");
            } else {
                RCLCPP_INFO(this->get_logger(), "Subscription is intact");
            }
            
        } else if (msg->data == "CORRUPT") {
            if (!corrupted_) {
                corrupt_subscription();
            } else {
                RCLCPP_WARN(this->get_logger(), "Already corrupted!");
            }
            
        } else if (msg->data == "TEST") {
            RCLCPP_INFO(this->get_logger(), "Test message processed");
            
            if (corrupted_) {
                RCLCPP_INFO(this->get_logger(), "If CFI is active, it should have detected the corruption!");
            }
            
        } else {
            RCLCPP_INFO(this->get_logger(), "Normal processing: %s", msg->data.c_str());
        }
    }
    
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    void* sub_ptr_;
    std::vector<unsigned char> original_bytes_;
    bool corrupted_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    
    std::cout << "\n";
    std::cout << "╔════════════════════════════════════════════╗\n";
    std::cout << "║     CFI Attack/Defense Demonstration       ║\n";
    std::cout << "╚════════════════════════════════════════════╝\n";
    std::cout << "\n";
    std::cout << "This demo shows how CFI protects against\n";
    std::cout << "memory corruption attacks on ROS2 callbacks.\n";
    std::cout << "\n";
    std::cout << "Attack sequence:\n";
    std::cout << "1. Send 'CHECK' - See normal state\n";
    std::cout << "2. Send 'CORRUPT' - Corrupt subscription\n";
    std::cout << "3. Send 'TEST' - Trigger corrupted callback\n";
    std::cout << "\n";
    std::cout << "Without CFI: May crash or behave unexpectedly\n";
    std::cout << "With CFI: Detects corruption and blocks execution\n";
    std::cout << "\n";
    
    auto node = std::make_shared<FinalAttackDemo>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
}
