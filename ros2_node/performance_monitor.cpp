#include <chrono>
#include <memory>
#include <vector>
#include <algorithm>
#include <numeric>
#include <iomanip>
#include <fstream>
#include <sys/resource.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono;
using namespace std::chrono_literals;

// 정밀한 성능 측정을 위한 모니터 노드
class PerformanceMonitor : public rclcpp::Node
{
public:
    PerformanceMonitor() : Node("performance_monitor")
    {
        RCLCPP_INFO(this->get_logger(), "=== CFI Performance Monitor ===");
        
        // 고정밀 타이머 설정
        this->declare_parameter("measurement_duration_sec", 60);
        this->declare_parameter("callback_frequency_hz", 1000);
        this->declare_parameter("warmup_iterations", 1000);
        
        measurement_duration_ = this->get_parameter("measurement_duration_sec").as_int();
        callback_frequency_ = this->get_parameter("callback_frequency_hz").as_int();
        warmup_iterations_ = this->get_parameter("warmup_iterations").as_int();
        
        // 측정용 subscription 생성
        perf_sub_ = this->create_subscription<std_msgs::msg::String>(
            "perf_topic", 
            rclcpp::QoS(10).reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE),
            std::bind(&PerformanceMonitor::perf_callback, this, std::placeholders::_1)
        );
        
        // 결과 publisher
        results_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "perf_results", 10
        );
        
        // 측정 시작 타이머
        start_timer_ = this->create_wall_timer(
            2s, std::bind(&PerformanceMonitor::start_measurement, this)
        );
        
        // 통계 계산 타이머
        stats_timer_ = this->create_wall_timer(
            1s, std::bind(&PerformanceMonitor::calculate_statistics, this)
        );
        
        // 메모리 사용량 추적 타이머
        memory_timer_ = this->create_wall_timer(
            100ms, std::bind(&PerformanceMonitor::track_memory_usage, this)
        );
        
        // 벡터 예약
        callback_latencies_.reserve(100000);
        cpu_usage_samples_.reserve(1000);
        memory_usage_samples_.reserve(1000);
        
        RCLCPP_INFO(this->get_logger(), 
            "Monitor configured: %d sec duration, %d Hz frequency",
            measurement_duration_, callback_frequency_);
    }
    
    ~PerformanceMonitor()
    {
        save_results_to_file();
    }

private:
    // 성능 측정 콜백
    void perf_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        auto callback_start = high_resolution_clock::now();
        
        callback_count_++;
        
        // Warmup 기간 스킵
        if (callback_count_ < warmup_iterations_) {
            if (callback_count_ == warmup_iterations_) {
                RCLCPP_INFO(this->get_logger(), "Warmup complete, starting measurement");
                measurement_start_time_ = high_resolution_clock::now();
                last_callback_time_ = measurement_start_time_;
            }
            return;
        }
        
        // 콜백 간격 측정
        auto now = high_resolution_clock::now();
        auto interval = duration_cast<nanoseconds>(now - last_callback_time_);
        last_callback_time_ = now;
        
        // 메시지 처리 시뮬레이션
        volatile size_t msg_len = msg->data.length();
        volatile double dummy = 0;
        for (int i = 0; i < 100; i++) {
            dummy += std::sin(i) * std::cos(i);
        }
        (void)msg_len;
        (void)dummy;
        
        // 콜백 레이턴시 측정
        auto callback_end = high_resolution_clock::now();
        auto latency = duration_cast<nanoseconds>(callback_end - callback_start);
        
        callback_latencies_.push_back(latency.count());
        callback_intervals_.push_back(interval.count());
        
        // 정기적 로그
        if (callback_count_ % 1000 == 0) {
            auto elapsed = duration_cast<seconds>(now - measurement_start_time_);
            RCLCPP_INFO(this->get_logger(), 
                "Progress: %zu callbacks, %ld seconds elapsed",
                callback_count_ - warmup_iterations_, elapsed.count());
        }
    }
    
    // 측정 시작
    void start_measurement()
    {
        RCLCPP_INFO(this->get_logger(), "Starting performance measurement...");
        
        // CPU 기준선 측정
        baseline_cpu_usage_ = get_cpu_usage();
        
        // 메시지 발행 시작
        msg_publisher_ = this->create_publisher<std_msgs::msg::String>("perf_topic", 10);
        msg_timer_ = this->create_wall_timer(
            std::chrono::microseconds(1000000 / callback_frequency_),
            [this]() {
                auto msg = std_msgs::msg::String();
                msg.data = "Performance test message " + std::to_string(msg_count_++);
                msg_publisher_->publish(msg);
            }
        );
        
        start_timer_.reset();  // 한 번만 실행
    }
    
    // 통계 계산
    void calculate_statistics()
    {
        if (callback_latencies_.size() < 100) return;
        
        // 레이턴시 통계
        auto latencies_copy = callback_latencies_;
        std::sort(latencies_copy.begin(), latencies_copy.end());
        
        double mean_latency = std::accumulate(latencies_copy.begin(), 
                                            latencies_copy.end(), 0.0) / latencies_copy.size();
        
        size_t p50_idx = latencies_copy.size() * 0.50;
        size_t p95_idx = latencies_copy.size() * 0.95;
        size_t p99_idx = latencies_copy.size() * 0.99;
        
        double p50_latency = latencies_copy[p50_idx] / 1000.0;  // μs
        double p95_latency = latencies_copy[p95_idx] / 1000.0;
        double p99_latency = latencies_copy[p99_idx] / 1000.0;
        double min_latency = latencies_copy.front() / 1000.0;
        double max_latency = latencies_copy.back() / 1000.0;
        
        // CPU 사용량
        double current_cpu = get_cpu_usage();
        cpu_usage_samples_.push_back(current_cpu - baseline_cpu_usage_);
        
        // 결과 발행
        auto result_msg = std_msgs::msg::Float64MultiArray();
        result_msg.data = {
            mean_latency / 1000.0,  // mean (μs)
            p50_latency,            // p50 (μs)
            p95_latency,            // p95 (μs)
            p99_latency,            // p99 (μs)
            min_latency,            // min (μs)
            max_latency,            // max (μs)
            current_cpu - baseline_cpu_usage_  // CPU overhead (%)
        };
        results_pub_->publish(result_msg);
        
        // 로그 출력
        RCLCPP_INFO(this->get_logger(), 
            "\n=== Performance Statistics ==="
            "\nLatency (μs): mean=%.2f, p50=%.2f, p95=%.2f, p99=%.2f"
            "\n              min=%.2f, max=%.2f"
            "\nCPU overhead: %.2f%%"
            "\nCallbacks: %zu",
            mean_latency / 1000.0, p50_latency, p95_latency, p99_latency,
            min_latency, max_latency,
            current_cpu - baseline_cpu_usage_,
            callback_count_ - warmup_iterations_);
        
        // 측정 종료 체크
        if (callback_count_ > warmup_iterations_) {
            auto elapsed = duration_cast<seconds>(
                high_resolution_clock::now() - measurement_start_time_);
            if (elapsed.count() >= measurement_duration_) {
                RCLCPP_INFO(this->get_logger(), "Measurement complete!");
                save_results_to_file();
                rclcpp::shutdown();
            }
        }
    }
    
    // 메모리 사용량 추적
    void track_memory_usage()
    {
        struct rusage usage;
        if (getrusage(RUSAGE_SELF, &usage) == 0) {
            // RSS in KB
            long rss_kb = usage.ru_maxrss;
            memory_usage_samples_.push_back(rss_kb);
            
            if (memory_usage_samples_.size() == 1) {
                baseline_memory_kb_ = rss_kb;
            }
        }
    }
    
    // CPU 사용량 측정
    double get_cpu_usage()
    {
        static clock_t last_cpu = 0;
        static clock_t last_sys_cpu = 0;
        static clock_t last_user_cpu = 0;
        
        struct rusage usage;
        getrusage(RUSAGE_SELF, &usage);
        
        clock_t user_cpu = usage.ru_utime.tv_sec * 1000000 + usage.ru_utime.tv_usec;
        clock_t sys_cpu = usage.ru_stime.tv_sec * 1000000 + usage.ru_stime.tv_usec;
        
        double percent = 0.0;
        if (last_cpu != 0) {
            percent = ((user_cpu - last_user_cpu) + (sys_cpu - last_sys_cpu)) * 100.0 / 
                     (clock() - last_cpu);
        }
        
        last_cpu = clock();
        last_sys_cpu = sys_cpu;
        last_user_cpu = user_cpu;
        
        return percent;
    }
    
    // 결과 파일 저장
    void save_results_to_file()
    {
        std::ofstream file("/tmp/cfi_performance_results.csv");
        if (!file.is_open()) return;
        
        file << "CFI Performance Measurement Results\n";
        file << "Duration: " << measurement_duration_ << " seconds\n";
        file << "Callback frequency: " << callback_frequency_ << " Hz\n";
        file << "Total callbacks: " << callback_count_ - warmup_iterations_ << "\n\n";
        
        // 레이턴시 통계
        if (!callback_latencies_.empty()) {
            auto latencies_copy = callback_latencies_;
            std::sort(latencies_copy.begin(), latencies_copy.end());
            
            file << "Latency Statistics (microseconds):\n";
            file << "Mean," << std::accumulate(latencies_copy.begin(), 
                                              latencies_copy.end(), 0.0) / 
                                              latencies_copy.size() / 1000.0 << "\n";
            file << "P50," << latencies_copy[latencies_copy.size() * 0.50] / 1000.0 << "\n";
            file << "P95," << latencies_copy[latencies_copy.size() * 0.95] / 1000.0 << "\n";
            file << "P99," << latencies_copy[latencies_copy.size() * 0.99] / 1000.0 << "\n";
            file << "Min," << latencies_copy.front() / 1000.0 << "\n";
            file << "Max," << latencies_copy.back() / 1000.0 << "\n\n";
        }
        
        // CPU 사용량
        if (!cpu_usage_samples_.empty()) {
            double avg_cpu = std::accumulate(cpu_usage_samples_.begin(), 
                                           cpu_usage_samples_.end(), 0.0) / 
                                           cpu_usage_samples_.size();
            file << "CPU Overhead: " << avg_cpu << "%\n";
        }
        
        // 메모리 사용량
        if (!memory_usage_samples_.empty()) {
            long max_memory = *std::max_element(memory_usage_samples_.begin(), 
                                              memory_usage_samples_.end());
            file << "Memory Overhead: " << (max_memory - baseline_memory_kb_) 
                 << " KB\n";
        }
        
        file << "\nRaw latency data (first 1000 samples in nanoseconds):\n";
        for (size_t i = 0; i < std::min(size_t(1000), callback_latencies_.size()); i++) {
            file << callback_latencies_[i] << "\n";
        }
        
        file.close();
        RCLCPP_INFO(this->get_logger(), 
            "Results saved to /tmp/cfi_performance_results.csv");
    }
    
    // 멤버 변수들
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr perf_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr msg_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr results_pub_;
    
    rclcpp::TimerBase::SharedPtr start_timer_;
    rclcpp::TimerBase::SharedPtr msg_timer_;
    rclcpp::TimerBase::SharedPtr stats_timer_;
    rclcpp::TimerBase::SharedPtr memory_timer_;
    
    std::vector<long> callback_latencies_;
    std::vector<long> callback_intervals_;
    std::vector<double> cpu_usage_samples_;
    std::vector<long> memory_usage_samples_;
    
    size_t callback_count_ = 0;
    size_t msg_count_ = 0;
    int measurement_duration_;
    int callback_frequency_;
    int warmup_iterations_;
    
    high_resolution_clock::time_point measurement_start_time_;
    high_resolution_clock::time_point last_callback_time_;
    
    double baseline_cpu_usage_ = 0.0;
    long baseline_memory_kb_ = 0;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<PerformanceMonitor>();
    
    RCLCPP_INFO(node->get_logger(), 
        "Performance monitor started. This will run for the configured duration.");
    
    try {
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception: %s", e.what());
    }
    
    return 0;
}
