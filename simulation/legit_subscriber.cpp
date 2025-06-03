// legit_subscriber.cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class LegitSubscriber : public rclcpp::Node
{
public:
  LegitSubscriber()
  : Node("legit_subscriber")
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "secret_data", 10,
      std::bind(&LegitSubscriber::topic_callback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "[LegitSubscriber] Subscribed to 'secret_data'");
  }

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "[LegitSubscriber] Received: '%s'", msg->data.c_str());
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LegitSubscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

