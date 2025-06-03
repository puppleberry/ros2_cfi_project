// attack_subscriber.cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class AttackSubscriber : public rclcpp::Node
{
public:
  AttackSubscriber()
  : Node("attack_subscriber")
  {
    // 1) “secret_data” 토픽으로 구독자 생성
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "secret_data", 10,
      std::bind(&AttackSubscriber::topic_callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "[AttackSubscriber] Attempting to subscribe to 'secret_data' ...");
  }

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_WARN(this->get_logger(), "[AttackSubscriber] GOT SECRET: '%s'", msg->data.c_str());
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AttackSubscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

