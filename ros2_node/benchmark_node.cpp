#include <chrono>
#include <memory>
#include <thread>
#include <atomic>
#include <iostream>
#include <vector>
#include <dlfcn.h>  // dlopen, dlsym, dlclose를 위해 추가
#include <sstream>  // stringstream을 위해 추가

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;

class BenchmarkNode : public rclcpp::Node
{
public:
  BenchmarkNode() : Node("benchmark_node")
  {
    RCLCPP_INFO(this->get_logger(), "=== CFI Performance Benchmark Node ===");
    
    // 여러 subscription 생성 (부하 테스트)
    for (int i = 0; i < 10; i++) {
      auto topic_name = "benchmark_topic_" + std::to_string(i);
      
      subscriptions_.push_back(
        this->create_subscription<std_msgs::msg::String>(
          topic_name, 10,
          [this, i](const std_msgs::msg::String::SharedPtr msg) {
            this->message_callback(msg, i);
          }
        )
      );
      
      RCLCPP_INFO(this->get_logger(), "Created subscription for %s", topic_name.c_str());
    }
    
    // 통계 수집용 subscription
    stats_subscription_ = this->create_subscription<std_msgs::msg::String>(
      "cfi_stats_request", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        (void)msg;  // unused parameter
        this->print_statistics();
      }
    );
    
    // 부하 생성용 publisher
    for (int i = 0; i < 10; i++) {
      auto topic_name = "benchmark_topic_" + std::to_string(i);
      publishers_.push_back(
        this->create_publisher<std_msgs::msg::String>(topic_name, 10)
      );
    }
    
    // 주기적으로 메시지 발행
    timer_ = this->create_wall_timer(
      10ms,  // 100Hz로 메시지 발행
      [this]() { this->timer_callback(); }
    );
    
    // 통계 출력 타이머
    stats_timer_ = this->create_wall_timer(
      5s,
      [this]() { this->print_statistics(); }
    );
    
    start_time_ = std::chrono::steady_clock::now();
  }

private:
  void message_callback(const std_msgs::msg::String::SharedPtr msg, int topic_id)
  {
    callback_count_++;
    
    // 메시지 내용도 활용 (unused parameter 경고 제거)
    volatile size_t msg_len = msg->data.length();
    (void)msg_len;  // 경고 방지
    
    // CPU 부하 시뮬레이션 (간단한 계산)
    volatile double result = 0;
    for (int i = 0; i < 100; i++) {
      result += std::sin(i) * std::cos(i);
    }
    
    // 가끔 로그 출력
    if (callback_count_ % 1000 == 0) {
      RCLCPP_INFO(this->get_logger(), 
        "Processed %ld messages (topic %d)", 
        callback_count_.load(), topic_id);
    }
  }
  
  void timer_callback()
  {
    static size_t counter = 0;
    
    // 각 publisher로 메시지 발행
    auto message = std_msgs::msg::String();
    message.data = "Benchmark message " + std::to_string(counter++);
    
    for (size_t i = 0; i < publishers_.size(); i++) {
      // 일부 토픽에만 발행하여 불균형 부하 생성
      if (counter % (i + 1) == 0) {
        publishers_[i]->publish(message);
        publish_count_++;
      }
    }
  }
  
  void print_statistics()
  {
    auto now = std::chrono::steady_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - start_time_);
    
    double seconds = duration.count();
    double callback_rate = callback_count_ / seconds;
    double publish_rate = publish_count_ / seconds;
    
    RCLCPP_INFO(this->get_logger(), 
      "\n=== Benchmark Statistics ==="
      "\nRuntime: %.1f seconds"
      "\nCallbacks processed: %ld (%.1f Hz)"
      "\nMessages published: %ld (%.1f Hz)"
      "\nSubscriptions: %zu"
      "\nPublishers: %zu",
      seconds, 
      callback_count_.load(), callback_rate,
      publish_count_.load(), publish_rate,
      subscriptions_.size(), publishers_.size()
    );
    
    // CFI 통계 요청 (외부 함수 호출)
    typedef void (*cfi_stats_func)();
    void* handle = dlopen(nullptr, RTLD_LAZY);
    if (handle) {
      cfi_stats_func print_cfi_stats = 
        (cfi_stats_func)dlsym(handle, "cfi_print_performance_stats");
      if (print_cfi_stats) {
        print_cfi_stats();
      }
      dlclose(handle);
    }
  }
  
  // 멤버 변수들
  std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> subscriptions_;
  std::vector<rclcpp::Publisher<std_msgs::msg::String>::SharedPtr> publishers_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr stats_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
  
  std::atomic<size_t> callback_count_{0};
  std::atomic<size_t> publish_count_{0};
  std::chrono::steady_clock::time_point start_time_;
};

// 멀티스레드 벤치마크 노드
class MultiThreadBenchmarkNode : public rclcpp::Node
{
public:
  MultiThreadBenchmarkNode() : Node("multithread_benchmark_node")
  {
    RCLCPP_INFO(this->get_logger(), "=== Multi-threaded Benchmark Node ===");
    
    // 여러 콜백 그룹 생성
    for (int i = 0; i < 4; i++) {
      callback_groups_.push_back(
        this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive)
      );
    }
    
    // 각 그룹에 subscription 할당
    for (int i = 0; i < 16; i++) {
      auto topic_name = "mt_benchmark_topic_" + std::to_string(i);
      auto callback_group = callback_groups_[i % 4];
      
      rclcpp::SubscriptionOptions options;
      options.callback_group = callback_group;
      
      mt_subscriptions_.push_back(
        this->create_subscription<std_msgs::msg::String>(
          topic_name, 10,
          [this, i](const std_msgs::msg::String::SharedPtr msg) {
            this->mt_callback(msg, i);
          },
          options
        )
      );
    }
    
    RCLCPP_INFO(this->get_logger(), 
      "Created %zu subscriptions across %zu callback groups",
      mt_subscriptions_.size(), callback_groups_.size());
  }
  
private:
  void mt_callback(const std_msgs::msg::String::SharedPtr msg, int id)
  {
    mt_callback_count_++;
    
    // 메시지 활용
    volatile size_t msg_len = msg->data.length();
    (void)msg_len;  // 경고 방지
    
    // 스레드 정보 로깅
    if (mt_callback_count_ % 1000 == 0) {
      std::stringstream ss;
      ss << std::this_thread::get_id();
      RCLCPP_INFO(this->get_logger(), 
        "Thread %s processed callback %d (total: %ld)",
        ss.str().c_str(), id, mt_callback_count_.load());
    }
  }
  
  std::vector<rclcpp::CallbackGroup::SharedPtr> callback_groups_;
  std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> mt_subscriptions_;
  std::atomic<size_t> mt_callback_count_{0};
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  
  // 명령행 인자로 모드 선택
  bool multithread = false;
  if (argc > 1 && std::string(argv[1]) == "--multithread") {
    multithread = true;
  }
  
  if (multithread) {
    // 멀티스레드 executor 사용
    auto node = std::make_shared<MultiThreadBenchmarkNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    
    RCLCPP_INFO(node->get_logger(), "Running with MultiThreadedExecutor");
    executor.spin();
  } else {
    // 단일 스레드 실행
    auto node = std::make_shared<BenchmarkNode>();
    RCLCPP_INFO(node->get_logger(), "Running with SingleThreadedExecutor");
    rclcpp::spin(node);
  }
  
  rclcpp::shutdown();
  return 0;
}
