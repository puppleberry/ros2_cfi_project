#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* 이 클래스는 ROS2의 기본 Node 클래스를 상속받아 Publisher 기능을 구현합니다.
   나중에 CFI 프로젝트에서 이런 노드들의 콜백 메커니즘을 분석하게 됩니다. */
class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    // Publisher 객체를 생성합니다. 여기서 중요한 것은 토픽 이름("topic")과 
    // 큐 크기(10)입니다. 큐 크기는 메시지가 쌓일 수 있는 최대 개수를 의미합니다.
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    
    // 타이머를 생성하여 주기적으로 메시지를 발행합니다.
    // 500ms마다 timer_callback 함수가 호출됩니다.
    // 이 타이머 콜백이 바로 우리가 CFI에서 보호해야 할 함수 포인터 중 하나입니다.
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  // 타이머 콜백 함수 - 주기적으로 호출되어 메시지를 발행합니다.
  // 이 함수의 주소가 내부적으로 함수 포인터로 저장되며,
  // 공격자가 이를 변조할 수 있다면 임의의 코드를 실행할 수 있습니다.
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    
    // 실제 메시지를 네트워크에 발행합니다.
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  
  // 타이머와 publisher 객체를 멤버 변수로 저장합니다.
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  // ROS2 시스템을 초기화합니다.
  rclcpp::init(argc, argv);
  
  // 노드를 생성하고 실행합니다.
  // spin() 함수는 이벤트 루프를 시작하여 콜백들이 처리되도록 합니다.
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  
  // 프로그램 종료 시 ROS2 시스템을 정리합니다.
  rclcpp::shutdown();
  return 0;
}
