#include <memory>
#include <iostream>
#include <iomanip>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

/* 이 클래스는 rclcpp 내부의 메모리 구조를 분석하기 위한 특별한 노드입니다.
   콜백 함수 포인터가 메모리에서 어떻게 저장되는지 관찰할 수 있습니다. */
class MemoryAnalysisNode : public rclcpp::Node
{
public:
  MemoryAnalysisNode() : Node("memory_analysis")
  {
    // 먼저 이 객체(this)의 메모리 주소를 출력합니다.
    std::cout << "\n=== 메모리 레이아웃 분석 ===" << std::endl;
    std::cout << "Node 객체 주소: " << std::hex << this << std::endl;
    
    // Subscription 객체를 생성하기 전에 스택의 상태를 기록합니다.
    void* stack_marker = &stack_marker;
    std::cout << "스택 위치 마커: " << std::hex << stack_marker << std::endl;
    
    // 콜백 함수의 주소를 미리 확인해봅니다.
    // 이는 나중에 공격자가 이 주소를 악성 코드 주소로 바꾸려 할 부분입니다.
    auto callback_func = &MemoryAnalysisNode::message_callback;
    std::cout << "콜백 함수 주소: " << std::hex << reinterpret_cast<void*&>(callback_func) << std::endl;
    
    // Subscription 객체를 생성합니다.
    // 여기서 내부적으로 콜백 함수 포인터가 어딘가에 저장됩니다.
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "analysis_topic", 
      10, 
      std::bind(&MemoryAnalysisNode::message_callback, this, std::placeholders::_1)
    );
    
    // Subscription 객체의 메모리 주소를 확인합니다.
    std::cout << "Subscription 객체 주소: " << std::hex << subscription_.get() << std::endl;
    
    // Subscription 객체의 크기를 확인해봅니다.
    // 이를 통해 내부에 어떤 데이터들이 저장되어 있는지 추정할 수 있습니다.
    std::cout << "Subscription 객체 크기: " << sizeof(*subscription_) << " bytes" << std::endl;
    
    // 메모리 덤프를 위해 객체의 첫 부분을 살펴봅니다.
    // 주의: 이는 실험적 목적이며, 실제 프로덕션 코드에서는 사용하면 안 됩니다.
    this->analyze_subscription_memory();
  }

private:
  void message_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    // 콜백이 호출될 때마다 스택과 힙의 상태를 출력합니다.
    std::cout << "\n=== 스택 메모리 정보 ===" << std::endl;
    const void* local_var = &msg;
    std::cout << "메시지 포인터 주소: " << local_var << std::endl;
    std::cout << "로컬 변수 주소: " << &local_var << std::endl;
    std::cout << "함수 포인터 주소: " << 
                 reinterpret_cast<void*>(&MemoryAnalysisNode::message_callback) << std::endl;
  }
  
  void analyze_subscription_memory()
  {
    std::cout << "\n=== Subscription 메모리 분석 ===" << std::endl;
    
    // Subscription 객체의 메모리를 바이트 단위로 살펴봅니다.
    // 이를 통해 함수 포인터가 어느 오프셋에 저장되는지 파악할 수 있습니다.
    const unsigned char* memory_ptr = reinterpret_cast<const unsigned char*>(subscription_.get());
    
    std::cout << "Subscription 객체의 첫 64바이트:" << std::endl;
    for (size_t i = 0; i < 64 && i < sizeof(*subscription_); ++i) {
      if (i % 16 == 0) {
        std::cout << std::endl << std::hex << std::setfill('0') << std::setw(8) 
                  << (reinterpret_cast<uintptr_t>(memory_ptr) + i) << ": ";
      }
      std::cout << std::hex << std::setfill('0') << std::setw(2) 
                << static_cast<unsigned int>(memory_ptr[i]) << " ";
    }
    std::cout << std::endl;
    
    // 포인터 크기만큼의 데이터를 찾아서 잠재적 함수 포인터를 식별해봅니다.
    std::cout << "\n잠재적 포인터 값들:" << std::endl;
    for (size_t i = 0; i <= sizeof(*subscription_) - sizeof(void*); i += sizeof(void*)) {
      void* potential_ptr = *reinterpret_cast<void* const*>(memory_ptr + i);
      if (potential_ptr != nullptr) {
        std::cout << "오프셋 " << std::dec << i << ": " << std::hex << potential_ptr << std::endl;
      }
    }
  }
  
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  
  std::cout << "메모리 분석 노드를 시작합니다..." << std::endl;
  
  auto node = std::make_shared<MemoryAnalysisNode>();
  
  std::cout << "\n메시지를 기다리는 중... (다른 터미널에서 publisher 실행)" << std::endl;
  std::cout << "분석을 위해 다음 명령어를 실행하세요:" << std::endl;
  std::cout << "ros2 topic pub /analysis_topic std_msgs/msg/String \"data: 'test message'\"" << std::endl;
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
