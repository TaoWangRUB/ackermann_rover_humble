#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rover_monitor/msg/jetson_status.hpp>

#include <string>
#include <vector>

namespace rover_monitor
{

class JetsonProbe : public rclcpp::Node
{
public:
  explicit JetsonProbe(const rclcpp::NodeOptions & options);

private:
  void on_timer();

  // Sysfs/procfs readers
  std::vector<float> read_cpu_usage();
  float read_gpu_usage();
  void read_temperatures(float & cpu, float & gpu, float & board);
  bool read_thermal_throttle();
  bool read_power_throttle();
  void read_memory(int32_t & used_mb, int32_t & total_mb, int32_t & swap_mb);
  float read_disk_free();
  float read_wifi_signal();
  int32_t read_uptime();
  std::string read_power_mode();

  // Publisher
  rclcpp::Publisher<rover_monitor::msg::JetsonStatus>::SharedPtr pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Callback group
  rclcpp::CallbackGroup::SharedPtr cb_group_;

  // CPU usage tracking (delta between reads)
  struct CpuTicks {
    uint64_t user{0}, nice{0}, system{0}, idle{0},
             iowait{0}, irq{0}, softirq{0}, steal{0};
    uint64_t total() const {
      return user + nice + system + idle + iowait + irq + softirq + steal;
    }
    uint64_t active() const { return total() - idle - iowait; }
  };
  std::vector<CpuTicks> prev_cpu_ticks_;

  // Cached power mode (read once at startup)
  std::string power_mode_;

  // Config
  int thermal_zone_cpu_{0};
  int thermal_zone_gpu_{1};
  int thermal_zone_board_{2};
  std::string gpu_load_path_;
  std::vector<std::string> throttle_filter_;
};

}  // namespace rover_monitor
