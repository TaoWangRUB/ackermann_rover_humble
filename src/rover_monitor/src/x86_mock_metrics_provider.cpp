#include "rover_monitor/x86_mock_metrics_provider.hpp"
#include <rover_monitor/msg/jetson_status.hpp>

#include <fstream>
#include <sstream>
#include <filesystem>
#include <sys/statvfs.h>
#include <rclcpp/rclcpp.hpp>

namespace rover_monitor
{

X86MockMetricsProvider::X86MockMetricsProvider(rclcpp::Node * node)
: node_(node)
{
  // Initialize CPU ticks baseline
  read_cpu_usage();
}

rover_monitor::msg::JetsonStatus X86MockMetricsProvider::read_metrics()
  {
    auto status = rover_monitor::msg::JetsonStatus();

    // === REAL reads from x86 /proc ===
    status.cpu_usage_pct = read_cpu_usage();
    status.wifi_signal_dbm = read_wifi_signal();

    int32_t ram_used, ram_total, swap_used;
    read_memory(ram_used, ram_total, swap_used);
    status.ram_used_mb = ram_used;
    status.ram_total_mb = ram_total;
    status.swap_used_mb = swap_used;

    status.disk_free_gb = read_disk_free();
    status.uptime_s = read_uptime();

    // === SIMULATED Jetson-specific metrics (with optional overrides) ===
    status.gpu_usage_pct = 35.0f;  // Representative SLAM load

    // Read temperature overrides if available (for testing)
    status.temp_cpu_c = get_mock_parameter("temp_cpu_c", 52.0f);
    status.temp_gpu_c = get_mock_parameter("temp_gpu_c", 48.0f);
    status.temp_board_c = get_mock_parameter("temp_board_c", 45.0f);

    status.is_thermal_throttled = get_mock_parameter("is_thermal_throttled", false);
    status.is_power_throttled = get_mock_parameter("is_power_throttled", false);
    status.power_mode = "20W";

    // Error code priority (same logic as JetsonMetricsProvider)
    if (status.temp_cpu_c > 80.0f) {
      status.error_code = 1;
      status.error_msg = "CPU over-temperature (>80°C)";
    } else if (status.temp_gpu_c > 83.0f) {
      status.error_code = 2;
      status.error_msg = "GPU over-temperature (>83°C)";
    } else if (status.ram_total_mb > 0 &&
      static_cast<float>(status.ram_used_mb) / static_cast<float>(status.ram_total_mb) > 0.90f)
    {
      status.error_code = 3;
      status.error_msg = "RAM usage above 90%";
    } else if (status.disk_free_gb < 1.0f) {
      status.error_code = 4;
      status.error_msg = "Disk space critical (<1 GB)";
    } else if (status.is_thermal_throttled) {
      status.error_code = 5;
      status.error_msg = "Xavier thermal throttling active";
    } else if (status.is_power_throttled) {
      status.error_code = 6;
      status.error_msg = "Xavier power throttling active";
    } else {
      status.error_code = 0;
      status.error_msg = "";
    }

    return status;
}

std::vector<float> X86MockMetricsProvider::read_cpu_usage()
  {
    std::vector<float> usage;
    std::vector<CpuTicks> current_ticks;

    std::ifstream f("/proc/stat");
    if (!f.is_open()) { return usage; }

    std::string line;
    while (std::getline(f, line)) {
      if (line.substr(0, 3) != "cpu" || line[3] == ' ') { continue; }

      CpuTicks t;
      std::istringstream iss(line);
      std::string label;
      iss >> label >> t.user >> t.nice >> t.system >> t.idle
          >> t.iowait >> t.irq >> t.softirq >> t.steal;
      current_ticks.push_back(t);
    }

    if (prev_cpu_ticks_.size() == current_ticks.size()) {
      for (size_t i = 0; i < current_ticks.size(); ++i) {
        auto dt = current_ticks[i].total() - prev_cpu_ticks_[i].total();
        auto da = current_ticks[i].active() - prev_cpu_ticks_[i].active();
        float pct = (dt > 0) ? (static_cast<float>(da) / static_cast<float>(dt) * 100.0f)
                              : 0.0f;
        usage.push_back(pct);
      }
    }

    prev_cpu_ticks_ = current_ticks;
    return usage;
  }

void X86MockMetricsProvider::read_memory(int32_t & used_mb, int32_t & total_mb, int32_t & swap_mb)
{
    used_mb = 0;
    total_mb = 0;
    swap_mb = 0;

    std::ifstream f("/proc/meminfo");
    if (!f.is_open()) { return; }

    int64_t mem_total_kb = 0, mem_available_kb = 0;
    int64_t swap_total_kb = 0, swap_free_kb = 0;

    std::string line;
    while (std::getline(f, line)) {
      std::istringstream iss(line);
      std::string key;
      int64_t val;
      iss >> key >> val;

      if (key == "MemTotal:") { mem_total_kb = val; }
      else if (key == "MemAvailable:") { mem_available_kb = val; }
      else if (key == "SwapTotal:") { swap_total_kb = val; }
      else if (key == "SwapFree:") { swap_free_kb = val; }
    }

    total_mb = static_cast<int32_t>(mem_total_kb / 1024);
    used_mb = static_cast<int32_t>((mem_total_kb - mem_available_kb) / 1024);
    swap_mb = static_cast<int32_t>((swap_total_kb - swap_free_kb) / 1024);
  }

float X86MockMetricsProvider::read_disk_free()
{
    struct statvfs stat;
    if (statvfs("/", &stat) != 0) { return -1.0f; }
    return static_cast<float>(stat.f_bavail * stat.f_frsize) /
      (1024.0f * 1024.0f * 1024.0f);
  }

float X86MockMetricsProvider::read_wifi_signal()
{
    std::ifstream f("/proc/net/wireless");
    if (!f.is_open()) { return -55.0f; }  // Nominal signal if not available

    std::string line;
    std::getline(f, line);
    std::getline(f, line);

    if (std::getline(f, line)) {
      std::istringstream iss(line);
      std::string iface;
      int status_val;
      float link, level;
      iss >> iface >> status_val >> link >> level;
      return level;
    }
    return -55.0f;  // Nominal signal if parse failed
  }

int32_t X86MockMetricsProvider::read_uptime()
{
    std::ifstream f("/proc/uptime");
    if (!f.is_open()) { return 0; }
    double uptime = 0.0;
    f >> uptime;
    return static_cast<int32_t>(uptime);
  }

float X86MockMetricsProvider::get_mock_parameter(const std::string & param_name, float default_value)
{
  if (!node_) { return default_value; }
  try {
    std::string full_param = "mock." + param_name;
    if (!node_->has_parameter(full_param)) {
      return default_value;
    }
    return node_->get_parameter(full_param).as_double();
  } catch (...) {
    return default_value;
  }
}

bool X86MockMetricsProvider::get_mock_parameter(const std::string & param_name, bool default_value)
{
  if (!node_) { return default_value; }
  try {
    std::string full_param = "mock." + param_name;
    if (!node_->has_parameter(full_param)) {
      return default_value;
    }
    return node_->get_parameter(full_param).as_bool();
  } catch (...) {
    return default_value;
  }
}

}  // namespace rover_monitor
