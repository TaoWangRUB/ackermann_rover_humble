#ifndef ROVER_MONITOR_X86_MOCK_METRICS_PROVIDER_HPP
#define ROVER_MONITOR_X86_MOCK_METRICS_PROVIDER_HPP

#include "rover_monitor/system_metrics_provider.hpp"
#include <rover_monitor/msg/jetson_status.hpp>

#include <string>
#include <vector>
#include <cstdint>
#include <rclcpp/node.hpp>

namespace rover_monitor
{

/**
 * @brief x86_64 development machine metrics provider (mock Jetson)
 *
 * REAL reads from /proc (for testing alert rules against actual machine load):
 *   - cpu_usage_pct: real per-core CPU usage from /proc/stat delta
 *   - ram_used_mb, ram_total_mb, swap_used_mb: real memory from /proc/meminfo
 *   - disk_free_gb: real disk from statvfs
 *   - uptime_s: real uptime from /proc/uptime
 *   - wifi_signal_dbm: real WiFi from /proc/net/wireless (or nominal -55 dBm if not available)
 *
 * SIMULATED stable values (Jetson-specific metrics):
 *   - gpu_usage_pct: 35.0 (representative SLAM load, won't trigger alerts)
 *   - temp_cpu_c: 52.0 (safe margin below 80°C threshold)
 *   - temp_gpu_c: 48.0 (safe margin below 83°C threshold)
 *   - temp_board_c: 45.0
 *   - is_thermal_throttled: false
 *   - is_power_throttled: false
 *   - power_mode: "20W"
 *
 * Supports ROS parameter overrides for fault injection (testing alert rules)
 */
class X86MockMetricsProvider : public SystemMetricsProvider
{
public:
  explicit X86MockMetricsProvider(rclcpp::Node * node = nullptr);

  rover_monitor::msg::JetsonStatus read_metrics() override;
  bool is_jetson_hardware() const override { return false; }
  std::string platform_name() const override { return "x86_mock"; }

private:
  rclcpp::Node * node_;

  struct CpuTicks {
    uint64_t user{0}, nice{0}, system{0}, idle{0},
             iowait{0}, irq{0}, softirq{0}, steal{0};
    uint64_t total() const {
      return user + nice + system + idle + iowait + irq + softirq + steal;
    }
    uint64_t active() const { return total() - idle - iowait; }
  };
  std::vector<CpuTicks> prev_cpu_ticks_;

  std::vector<float> read_cpu_usage();
  void read_memory(int32_t & used_mb, int32_t & total_mb, int32_t & swap_mb);
  float read_disk_free();
  float read_wifi_signal();
  int32_t read_uptime();

  float get_mock_parameter(const std::string & param_name, float default_value);
  bool get_mock_parameter(const std::string & param_name, bool default_value);
};

}  // namespace rover_monitor

#endif  // ROVER_MONITOR_X86_MOCK_METRICS_PROVIDER_HPP
