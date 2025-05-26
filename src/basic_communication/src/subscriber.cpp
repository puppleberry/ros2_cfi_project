#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

/* 이 클래스는 메시지를 수신하는 Subscriber 노드입니다.
   CFI 관점에서 가장 중요한 부분은 topic_callback 함수 포인터가 
   어떻게 저장되고 호출되는지입니다. */
class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    // Subscription 객체를 생성합니다. 여기서 핵심은 콜백 함수의 바인딩입니다.
    // std::bind를 통해 topic_callback 멤버 함수가 함수 포인터로 변환되어
    // ROS2 내부 구조에 저장됩니다. 이 함수 포인터가 공격의 타겟이 됩니다.
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:
  // 메시지 수신 콜백 함수 - 이것이 바로 우리가 CFI로 보호해야 할 함수입니다.
  // 공격자가 이 함수의 주소를 악성 코드 주소로 변경할 수 있다면,
  // 메시지가 도착할 때마다 악성 코드가 실행됩니다.
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  }
  
  // Subscription 객체를 멤버 변수로 저장합니다.
  // 이 객체 내부에 콜백 함수 포인터가 저장되어 있습니다.
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  // ROS2 시스템을 초기화합니다.
  rclcpp::init(argc, argv);
  
  // 노드를 생성하고 이벤트 루프를 시작합니다.
  // spin() 함수가 실행되는 동안 메시지가 도착하면 topic_callback이 호출됩니다.
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  
  // 프로그램 종료 시 정리 작업을 수행합니다.
  rclcpp::shutdown();
  return 0;
}
