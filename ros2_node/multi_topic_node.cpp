#include <memory>
#include <vector>
#include <string>
#include <functional>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float64.hpp"

using std::placeholders::_1;

// 다중 토픽을 구독하는 복잡한 노드
// CFI가 여러 콜백을 동시에 관리할 수 있는지 테스트
class MultiTopicNode : public rclcpp::Node
{
public:
    MultiTopicNode() : Node("multi_topic_node")
    {
        RCLCPP_INFO(this->get_logger(), "=== Multi-Topic Test Node ===");
        RCLCPP_INFO(this->get_logger(), "This node tests CFI with multiple subscription types");
        
        // 다양한 타입의 subscription 생성
        // 1. String 타입 subscriptions (5개)
        for (int i = 0; i < 5; i++) {
            auto topic_name = "string_topic_" + std::to_string(i);
            string_subs_.push_back(
                this->create_subscription<std_msgs::msg::String>(
                    topic_name, 10,
                    [this, i](const std_msgs::msg::String::SharedPtr msg) {
                        this->string_callback(msg, i);
                    }
                )
            );
            RCLCPP_INFO(this->get_logger(), "Created string subscription: %s", topic_name.c_str());
        }
        
        // 2. Int32 타입 subscriptions (3개)
        for (int i = 0; i < 3; i++) {
            auto topic_name = "int_topic_" + std::to_string(i);
            int_subs_.push_back(
                this->create_subscription<std_msgs::msg::Int32>(
                    topic_name, 10,
                    [this, i](const std_msgs::msg::Int32::SharedPtr msg) {
                        this->int_callback(msg, i);
                    }
                )
            );
            RCLCPP_INFO(this->get_logger(), "Created int32 subscription: %s", topic_name.c_str());
        }
        
        // 3. Float64 타입 subscription
        float_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "float_topic", 10,
            std::bind(&MultiTopicNode::float_callback, this, _1)
        );
        
        // 4. 동적으로 추가/제거되는 subscriptions
        dynamic_timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&MultiTopicNode::manage_dynamic_subscriptions, this)
        );
        
        // 통계 출력 타이머
        stats_timer_ = this->create_wall_timer(
            std::chrono::seconds(10),
            std::bind(&MultiTopicNode::print_statistics, this)
        );
        
        RCLCPP_INFO(this->get_logger(), 
            "Total initial subscriptions: %zu", 
            string_subs_.size() + int_subs_.size() + 1);
    }

private:
    // String 메시지 콜백
    void string_callback(const std_msgs::msg::String::SharedPtr msg, int topic_id)
    {
        callback_counts_["string_" + std::to_string(topic_id)]++;
        
        // 메시지 처리 시뮬레이션
        if (msg->data.find("attack") != std::string::npos) {
            RCLCPP_WARN(this->get_logger(), 
                "Suspicious message on string_topic_%d: %s", 
                topic_id, msg->data.c_str());
        }
        
        // 가끔 로그 출력
        if (callback_counts_["string_" + std::to_string(topic_id)] % 100 == 0) {
            RCLCPP_INFO(this->get_logger(), 
                "String topic %d: %zu messages processed", 
                topic_id, callback_counts_["string_" + std::to_string(topic_id)]);
        }
    }
    
    // Int32 메시지 콜백
    void int_callback(const std_msgs::msg::Int32::SharedPtr msg, int topic_id)
    {
        callback_counts_["int_" + std::to_string(topic_id)]++;
        
        // 값 범위 검증
        if (msg->data < 0 || msg->data > 1000) {
            RCLCPP_WARN(this->get_logger(), 
                "Out of range value on int_topic_%d: %d", 
                topic_id, msg->data);
        }
        
        // 통계 업데이트
        total_int_sum_ += msg->data;
    }
    
    // Float64 메시지 콜백
    void float_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        callback_counts_["float"]++;
        
        // NaN 또는 Inf 체크
        if (std::isnan(msg->data) || std::isinf(msg->data)) {
            RCLCPP_ERROR(this->get_logger(), 
                "Invalid float value received: %f", msg->data);
        }
    }
    
    // 동적 subscription 관리
    void manage_dynamic_subscriptions()
    {
        dynamic_count_++;
        
        if (dynamic_count_ % 2 == 1) {
            // 새 subscription 추가
            auto topic_name = "dynamic_topic_" + std::to_string(dynamic_count_);
            dynamic_subs_.push_back(
                this->create_subscription<std_msgs::msg::String>(
                    topic_name, 10,
                    [this, topic_name](const std_msgs::msg::String::SharedPtr msg) {
                        callback_counts_[topic_name]++;
                        RCLCPP_INFO(this->get_logger(), 
                            "Dynamic topic %s: %s", 
                            topic_name.c_str(), msg->data.c_str());
                    }
                )
            );
            RCLCPP_INFO(this->get_logger(), 
                "✓ Added dynamic subscription: %s", topic_name.c_str());
        } else if (!dynamic_subs_.empty()) {
            // 마지막 subscription 제거
            dynamic_subs_.pop_back();
            RCLCPP_INFO(this->get_logger(), 
                "✗ Removed last dynamic subscription");
        }
        
        RCLCPP_INFO(this->get_logger(), 
            "Current dynamic subscriptions: %zu", dynamic_subs_.size());
    }
    
    // 통계 출력
    void print_statistics()
    {
        RCLCPP_INFO(this->get_logger(), "\n=== Multi-Topic Node Statistics ===");
        
        size_t total_callbacks = 0;
        for (const auto& [topic, count] : callback_counts_) {
            RCLCPP_INFO(this->get_logger(), "  %s: %zu callbacks", 
                topic.c_str(), count);
            total_callbacks += count;
        }
        
        RCLCPP_INFO(this->get_logger(), "Total callbacks: %zu", total_callbacks);
        RCLCPP_INFO(this->get_logger(), "Total int sum: %ld", total_int_sum_.load());
        RCLCPP_INFO(this->get_logger(), "Active subscriptions: %zu",
            string_subs_.size() + int_subs_.size() + 1 + dynamic_subs_.size());
        
        // CFI 상태 확인을 위한 마커
        RCLCPP_INFO(this->get_logger(), "CFI_CHECK: All callbacks executed safely");
    }
    
    // 멤버 변수들
    std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> string_subs_;
    std::vector<rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr> int_subs_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr float_sub_;
    std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> dynamic_subs_;
    
    rclcpp::TimerBase::SharedPtr dynamic_timer_;
    rclcpp::TimerBase::SharedPtr stats_timer_;
    
    std::map<std::string, size_t> callback_counts_;
    std::atomic<long> total_int_sum_{0};
    int dynamic_count_ = 0;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<MultiTopicNode>();
    
    RCLCPP_INFO(node->get_logger(), "Multi-topic node started. Press Ctrl+C to exit.");
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}
