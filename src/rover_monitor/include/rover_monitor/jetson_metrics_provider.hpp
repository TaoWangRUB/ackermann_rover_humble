#ifndef ROVER_MONITOR_JETSON_METRICS_PROVIDER_HPP
#define ROVER_MONITOR_JETSON_METRICS_PROVIDER_HPP

#include "rover_monitor/system_metrics_provider.hpp"
#include <rover_monitor/msg/jetson_status.hpp>

#include <string>
#include <vector>
#include <cstdint>

namespace rover_monitor
{

/**
 * @brief Real Jetson hardware metrics provider
 * Reads actual sysfs/procfs paths with graceful error handling
 */
class JetsonMetricsProvider : public SystemMetricsProvider
{
public:
  explicit JetsonMetricsProvider(
    int thermal_zone_cpu = 0,
    int thermal_zone_gpu = 1,
    int thermal_zone_board = 2,
    const std::string & gpu_load_path = "/sys/devices/gpu.0/load",
    const std::vector<std::string> & throttle_filter = {"CPU-therm", "GPU-therm"});

  rover_monitor::msg::JetsonStatus read_metrics() override;
  bool is_jetson_hardware() const override { return true; }
  std::string platform_name() const override;

private:
  struct CpuTicks {
    uint64_t user{0}, nice{0}, system{0}, idle{0},
             iowait{0}, irq{0}, softirq{0}, steal{0};
    uint64_t total() const {
      return user + nice + system + idle + iowait + irq + softirq + steal;
    }
    uint64_t active() const { return total() - idle - iowait; }
  };
  std::vector<CpuTicks> prev_cpu_ticks_;
  std::string power_mode_;

  int thermal_zone_cpu_;
  int thermal_zone_gpu_;
  int thermal_zone_board_;
  std::string gpu_load_path_;
  std::vector<std::string> throttle_filter_;

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
};

}  // namespace rover_monitor

#endif  // ROVER_MONITOR_JETSON_METRICS_PROVIDER_HPP
