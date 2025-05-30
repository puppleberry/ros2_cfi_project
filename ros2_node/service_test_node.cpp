#include <memory>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/set_bool.hpp"

using namespace std::chrono_literals;

// Service/Client 패턴에서 CFI 테스트
class ServiceTestNode : public rclcpp::Node
{
public:
    ServiceTestNode() : Node("service_test_node")
    {
        RCLCPP_INFO(this->get_logger(), "=== Service/Client CFI Test Node ===");
        
        // Service 서버들 생성
        // 1. Add Two Ints 서비스
        add_service_ = this->create_service<example_interfaces::srv::AddTwoInts>(
            "add_two_ints",
            std::bind(&ServiceTestNode::add_callback, this, 
                     std::placeholders::_1, std::placeholders::_2)
        );
        
        // 2. Trigger 서비스 (취약점 시뮬레이션)
        trigger_service_ = this->create_service<std_srvs::srv::Trigger>(
            "trigger_vulnerable",
            std::bind(&ServiceTestNode::trigger_callback, this,
                     std::placeholders::_1, std::placeholders::_2)
        );
        
        // 3. Set Bool 서비스
        set_bool_service_ = this->create_service<std_srvs::srv::SetBool>(
            "set_cfi_mode",
            std::bind(&ServiceTestNode::set_bool_callback, this,
                     std::placeholders::_1, std::placeholders::_2)
        );
        
        // Client들 생성
        add_client_ = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
        trigger_client_ = this->create_client<std_srvs::srv::Trigger>("trigger_vulnerable");
        
        // 주기적으로 서비스 호출 테스트
        timer_ = this->create_wall_timer(
            2s, std::bind(&ServiceTestNode::test_services, this)
        );
        
        RCLCPP_INFO(this->get_logger(), "Service servers and clients created");
        
        // 메모리 레이아웃 정보 (디버깅용)
        RCLCPP_INFO(this->get_logger(), "Service callback addresses:");
        RCLCPP_INFO(this->get_logger(), "  add_callback: %p", 
                    (void*)&ServiceTestNode::add_callback);
        RCLCPP_INFO(this->get_logger(), "  trigger_callback: %p", 
                    (void*)&ServiceTestNode::trigger_callback);
        RCLCPP_INFO(this->get_logger(), "  set_bool_callback: %p", 
                    (void*)&ServiceTestNode::set_bool_callback);
    }

private:
    // Add Two Ints 서비스 콜백
    void add_callback(
        const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
        std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response)
    {
        service_call_count_++;
        
        // 오버플로우 체크
        if (request->a > INT64_MAX - request->b) {
            RCLCPP_WARN(this->get_logger(), 
                "Integer overflow prevented! a=%ld, b=%ld", 
                request->a, request->b);
            response->sum = INT64_MAX;
        } else {
            response->sum = request->a + request->b;
        }
        
        RCLCPP_INFO(this->get_logger(), 
            "Add service called: %ld + %ld = %ld (call #%zu)",
            request->a, request->b, response->sum, service_call_count_);
    }
    
    // Trigger 서비스 콜백 (취약점 시뮬레이션)
    void trigger_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        (void)request;  // unused
        trigger_count_++;
        
        // 버퍼를 사용하여 취약점 시뮬레이션
        char buffer[32];
        snprintf(buffer, sizeof(buffer), "Trigger #%d", trigger_count_);
        
        // 특정 횟수에서 "취약한" 동작 시뮬레이션
        if (trigger_count_ % 10 == 0) {
            RCLCPP_WARN(this->get_logger(), 
                "Simulating vulnerable behavior at trigger #%d", trigger_count_);
            
            // 위험한 함수 포인터 사용 시뮬레이션
            void (*func_ptr)() = &ServiceTestNode::dangerous_function;
            
            // CFI가 이를 감지해야 함
            RCLCPP_INFO(this->get_logger(), 
                "Function pointer: %p", (void*)func_ptr);
        }
        
        response->success = true;
        response->message = std::string(buffer) + " - CFI should protect this";
        
        RCLCPP_INFO(this->get_logger(), 
            "Trigger service called #%d", trigger_count_);
    }
    
    // Set Bool 서비스 콜백
    void set_bool_callback(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        cfi_strict_mode_ = request->data;
        
        response->success = true;
        response->message = cfi_strict_mode_ ? 
            "CFI strict mode enabled" : "CFI permissive mode enabled";
        
        RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
    }
    
    // 위험한 함수 (공격 시뮬레이션용)
    static void dangerous_function()
    {
        std::cout << "⚠️  DANGEROUS FUNCTION CALLED! CFI should prevent this!" << std::endl;
    }
    
    // 주기적 서비스 테스트
    void test_services()
    {
        test_count_++;
        
        // Add 서비스 테스트
        if (add_client_->wait_for_service(1s)) {
            auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
            request->a = test_count_;
            request->b = test_count_ * 2;
            
            auto future = add_client_->async_send_request(request);
            RCLCPP_INFO(this->get_logger(), 
                "Sent add request: %ld + %ld", request->a, request->b);
        }
        
        // Trigger 서비스 테스트
        if (test_count_ % 3 == 0 && trigger_client_->wait_for_service(1s)) {
            auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
            auto future = trigger_client_->async_send_request(request);
            RCLCPP_INFO(this->get_logger(), "Sent trigger request");
        }
        
        // 통계 출력
        if (test_count_ % 10 == 0) {
            RCLCPP_INFO(this->get_logger(), 
                "\n=== Service Test Statistics ==="
                "\nTotal service calls: %zu"
                "\nTrigger calls: %d"
                "\nTest iterations: %d"
                "\nCFI mode: %s",
                service_call_count_, trigger_count_, test_count_,
                cfi_strict_mode_ ? "STRICT" : "PERMISSIVE");
        }
    }
    
    // 멤버 변수들
    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr add_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr trigger_service_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_bool_service_;
    
    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr add_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr trigger_client_;
    
    rclcpp::TimerBase::SharedPtr timer_;
    
    size_t service_call_count_ = 0;
    int trigger_count_ = 0;
    int test_count_ = 0;
    bool cfi_strict_mode_ = false;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<ServiceTestNode>();
    
    RCLCPP_INFO(node->get_logger(), 
        "Service test node started. Services available:");
    RCLCPP_INFO(node->get_logger(), "  - /add_two_ints");
    RCLCPP_INFO(node->get_logger(), "  - /trigger_vulnerable");
    RCLCPP_INFO(node->get_logger(), "  - /set_cfi_mode");
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}
