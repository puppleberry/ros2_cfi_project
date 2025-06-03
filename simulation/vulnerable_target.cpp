// vulnerable_target.cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <chrono>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // 1) 노드 생성
  auto node = rclcpp::Node::make_shared("vulnerable_publisher");

  // 2) /secret_data라는 이름의 퍼블리셔(publisher) 생성
  auto publisher = node->create_publisher<std_msgs::msg::String>("secret_data", 10);

  // 3) 1초마다 “민감 문자열”을 퍼블리시
  rclcpp::WallRate loop_rate(1s);
  int counter = 0;
  while (rclcpp::ok()) {
    auto message = std_msgs::msg::String();
    message.data = "TOP_SECRET_MESSAGE_" + std::to_string(counter++);
    RCLCPP_INFO(node->get_logger(), "[Publish] %s", message.data.c_str());
    publisher->publish(message);
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}

