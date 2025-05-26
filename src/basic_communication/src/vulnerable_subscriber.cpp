#include <functional>
#include <memory>
#include <cstring>
#include <iostream>
#include <iomanip>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

/* 경고: 이 코드는 의도적으로 보안 취약점을 포함하고 있습니다.
   교육 및 연구 목적으로만 사용되어야 하며, 절대 프로덕션 환경에서 사용하지 마세요. */

class VulnerableSubscriber : public rclcpp::Node
{
public:
  VulnerableSubscriber() : Node("vulnerable_subscriber")
  {
    RCLCPP_WARN(this->get_logger(), "경고: 이 노드는 의도적인 취약점을 포함합니다!");
    RCLCPP_WARN(this->get_logger(), "교육 목적으로만 사용하세요.");
    
    // 메모리 레이아웃을 예측 가능하게 만들기 위해 멤버 변수들의 순서를 신중히 배치합니다.
    // 일반적으로 컴파일러는 멤버 변수들을 선언 순서대로 메모리에 배치합니다.
    
    // 현재 객체의 메모리 주소와 멤버 변수들의 위치를 출력합니다.
    std::cout << "\n=== 취약한 노드 메모리 레이아웃 ===" << std::endl;
    std::cout << "VulnerableSubscriber 객체 주소: " << std::hex << this << std::endl;
    std::cout << "vulnerable_buffer 주소: " << std::hex << &vulnerable_buffer_ << std::endl;
    std::cout << "function_pointer 주소: " << std::hex << &function_pointer_ << std::endl;
    std::cout << "subscription 주소: " << std::hex << &subscription_ << std::endl;
    
    // 멤버 변수들 사이의 거리를 계산하여 overflow가 가능한지 확인합니다.
    ptrdiff_t buffer_to_func_ptr = reinterpret_cast<char*>(&function_pointer_) - 
                                   reinterpret_cast<char*>(&vulnerable_buffer_);
    std::cout << "버퍼에서 함수 포인터까지 거리: " << std::dec << buffer_to_func_ptr << " bytes" << std::endl;
    
    // 함수 포인터를 합법적인 함수로 초기화합니다.
    function_pointer_ = &VulnerableSubscriber::legitimate_function;
    std::cout << "합법적인 함수 주소: " << std::hex << 
                 reinterpret_cast<void*>(function_pointer_) << std::endl;
    
    // 공격자가 사용할 수 있는 악성 함수의 주소도 출력합니다.
    // 실제 공격에서는 이 정보를 다른 방법으로 얻어야 합니다.
    std::cout << "악성 함수 주소: " << std::hex << 
                 reinterpret_cast<void*>(&VulnerableSubscriber::malicious_function) << std::endl;
    
    // Subscription을 생성합니다. 이때 콜백도 등록됩니다.
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "vulnerable_topic", 
      10, 
      std::bind(&VulnerableSubscriber::vulnerable_callback, this, _1)
    );
    
    std::cout << "\n노드가 준비되었습니다. 다음 명령어로 메시지를 보내보세요:" << std::endl;
    std::cout << "정상 메시지: ros2 topic pub /vulnerable_topic std_msgs/msg/String \"data: 'Hello'\"" << std::endl;
    std::cout << "공격 메시지: ros2 topic pub /vulnerable_topic std_msgs/msg/String \"data: '" << 
                 create_attack_payload() << "'\"" << std::endl;
  }

private:
  // 취약점의 핵심: 고정 크기 버퍼
  // 이 버퍼는 의도적으로 작게 설정되어 overflow가 쉽게 발생하도록 합니다.
  char vulnerable_buffer_[32];
  
  // 이 함수 포인터가 공격의 타겟입니다.
  // buffer overflow를 통해 이 포인터의 값을 변조할 수 있습니다.
  void (VulnerableSubscriber::*function_pointer_)();
  
  // ROS2 subscription 객체
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

  // 메시지를 받을 때 호출되는 취약한 콜백 함수
  void vulnerable_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    std::cout << "\n=== 콜백 호출됨 ===" << std::endl;
    std::cout << "받은 메시지 길이: " << msg->data.length() << " bytes" << std::endl;
    std::cout << "메시지 내용: " << msg->data << std::endl;
    
    // 현재 버퍼와 함수 포인터의 상태를 출력합니다.
    std::cout << "overflow 이전 함수 포인터: " << std::hex << 
                 reinterpret_cast<void*>(function_pointer_) << std::endl;
    
    // 특별한 공격 페이로드인지 확인합니다 (HEX: 접두사로 시작하는 경우)
    std::string processed_data;
    if (msg->data.length() > 4 && msg->data.substr(0, 4) == "HEX:") {
      // 16진 문자열을 바이너리 데이터로 변환합니다
      processed_data = hex_string_to_binary(msg->data.substr(4));
      std::cout << "16진 데이터를 바이너리로 변환: " << processed_data.length() << " bytes" << std::endl;
    } else {
      // 일반 문자열 그대로 사용
      processed_data = msg->data;
    }
    
    // 여기가 취약점입니다!
    // 안전하지 않은 메모리 복사를 수행합니다.
    if (processed_data.length() <= sizeof(vulnerable_buffer_)) {
      std::memcpy(vulnerable_buffer_, processed_data.c_str(), processed_data.length());
    } else {
      // 의도적인 buffer overflow 발생!
      std::memcpy(vulnerable_buffer_, processed_data.c_str(), processed_data.length());
      std::cout << "⚠️  Buffer overflow 발생! 데이터 길이: " << processed_data.length() 
                << ", 버퍼 크기: " << sizeof(vulnerable_buffer_) << std::endl;
    }
    
    std::cout << "overflow 이후 함수 포인터: " << std::hex << 
                 reinterpret_cast<void*>(function_pointer_) << std::endl;
    
    // 버퍼의 내용을 16진수로 출력하여 overflow 상황을 시각화합니다.
    std::cout << "버퍼 내용 (16진수):" << std::endl;
    for (size_t i = 0; i < sizeof(vulnerable_buffer_); ++i) {
      if (i % 16 == 0) std::cout << std::endl;
      std::cout << std::hex << std::setfill('0') << std::setw(2) 
                << static_cast<unsigned char>(vulnerable_buffer_[i]) << " ";
    }
    std::cout << std::endl;
    
    // 함수 포인터를 호출합니다.
    // 만약 overflow로 인해 포인터가 변조되었다면, 악성 함수가 호출될 수 있습니다.
    std::cout << "함수 포인터 호출 시도..." << std::endl;
    try {
      (this->*function_pointer_)();
    } catch (const std::exception& e) {
      std::cout << "함수 호출 중 예외 발생: " << e.what() << std::endl;
    } catch (...) {
      std::cout << "알 수 없는 예외 발생" << std::endl;
    }
  }
  
  // 합법적인 함수 - 정상적으로 호출되어야 하는 함수입니다.
  void legitimate_function()
  {
    std::cout << "✓ 합법적인 함수가 호출되었습니다. 시스템이 정상 작동 중입니다." << std::endl;
    RCLCPP_INFO(this->get_logger(), "정상적인 처리가 완료되었습니다.");
  }
  
  // 악성 함수 - 공격자가 실행하려는 함수를 시뮬레이션합니다.
  // 실제 공격에서는 이보다 훨씬 위험한 작업을 수행할 수 있습니다.
  void malicious_function()
  {
    std::cout << "🚨 경고: 악성 함수가 호출되었습니다!" << std::endl;
    std::cout << "🚨 공격이 성공했습니다! 시스템이 손상되었을 수 있습니다." << std::endl;
    RCLCPP_ERROR(this->get_logger(), "보안 침해 감지! CFI가 필요합니다!");
    
    // 실제 공격에서는 여기서 다음과 같은 악성 행위들이 가능합니다:
    // - 시스템 명령어 실행 (system() 호출)
    // - 추가적인 메모리 손상
    // - 네트워크를 통한 정보 유출
    // - 다른 프로세스에 대한 공격
    
    std::cout << "악성 행위 시뮬레이션: 가상의 중요 데이터 접근 시도..." << std::endl;
  }
  
  // 공격 페이로드를 생성하는 헬퍼 함수
  std::string create_attack_payload()
  {
    // 함수 포인터를 악성 함수의 주소로 덮어쓰는 페이로드를 생성합니다.
    // ROS2 String 메시지의 제약으로 인해 16진 문자열 형태로 생성합니다.
    
    std::string payload = "HEX:";
    
    // 먼저 버퍼를 가득 채웁니다 (32바이트를 16진 문자열로)
    for (int i = 0; i < 32; ++i) {
      payload += "41"; // 'A'의 ASCII 값인 0x41
    }
    
    // 그 다음 악성 함수의 주소를 16진 문자열로 추가합니다.
    void (VulnerableSubscriber::*malicious_ptr)() = &VulnerableSubscriber::malicious_function;
    const unsigned char* ptr_bytes = reinterpret_cast<const unsigned char*>(&malicious_ptr);
    
    // 포인터를 바이트별로 16진 문자열로 변환합니다
    for (size_t i = 0; i < sizeof(malicious_ptr); ++i) {
      char hex_byte[3];
      sprintf(hex_byte, "%02x", ptr_bytes[i]);
      payload += hex_byte;
    }
    
    return payload;
  }
  
  // 16진 문자열을 바이너리 데이터로 변환하는 헬퍼 함수
  std::string hex_string_to_binary(const std::string& hex_str)
  {
    std::string binary_data;
    
    // 16진 문자열은 항상 짝수 길이여야 합니다 (한 바이트당 두 문자)
    for (size_t i = 0; i < hex_str.length(); i += 2) {
      if (i + 1 < hex_str.length()) {
        // 두 16진 문자를 한 바이트로 변환합니다
        std::string byte_str = hex_str.substr(i, 2);
        unsigned char byte_val = static_cast<unsigned char>(std::stoul(byte_str, nullptr, 16));
        binary_data.push_back(byte_val);
      }
    }
    
    std::cout << "16진 변환 결과: " << binary_data.length() << " bytes 생성됨" << std::endl;
    return binary_data;
  }
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  
  std::cout << "취약한 Subscriber 노드를 시작합니다..." << std::endl;
  std::cout << "경고: 이 프로그램은 교육 목적으로만 사용되어야 합니다!" << std::endl;
  
  auto node = std::make_shared<VulnerableSubscriber>();
  
  try {
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    std::cout << "노드 실행 중 예외 발생: " << e.what() << std::endl;
  }
  
  rclcpp::shutdown();
  return 0;
}
