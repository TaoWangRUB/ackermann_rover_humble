#include "rover_monitor/jetson_metrics_provider.hpp"
#include <rover_monitor/msg/jetson_status.hpp>

#include <fstream>
#include <sstream>
#include <filesystem>
#include <sys/statvfs.h>
#include <cstdlib>
#include <array>
#include <cstdio>

namespace rover_monitor
{

JetsonMetricsProvider::JetsonMetricsProvider(
  int thermal_zone_cpu,
  int thermal_zone_gpu,
  int thermal_zone_board,
  const std::string & gpu_load_path,
  const std::vector<std::string> & throttle_filter)
: thermal_zone_cpu_(thermal_zone_cpu),
  thermal_zone_gpu_(thermal_zone_gpu),
  thermal_zone_board_(thermal_zone_board),
  gpu_load_path_(gpu_load_path),
  throttle_filter_(throttle_filter)
{
  // Cache power mode once at startup
  power_mode_ = read_power_mode();
  // Initialize CPU ticks baseline
  read_cpu_usage();
}

rover_monitor::msg::JetsonStatus JetsonMetricsProvider::read_metrics()
  {
    auto status = rover_monitor::msg::JetsonStatus();

    status.cpu_usage_pct = read_cpu_usage();
    status.gpu_usage_pct = read_gpu_usage();

    float cpu_t, gpu_t, board_t;
    read_temperatures(cpu_t, gpu_t, board_t);
    status.temp_cpu_c = cpu_t;
    status.temp_gpu_c = gpu_t;
    status.temp_board_c = board_t;

    status.is_thermal_throttled = read_thermal_throttle();
    status.is_power_throttled = read_power_throttle();

    int32_t ram_used, ram_total, swap_used;
    read_memory(ram_used, ram_total, swap_used);
    status.ram_used_mb = ram_used;
    status.ram_total_mb = ram_total;
    status.swap_used_mb = swap_used;

    status.disk_free_gb = read_disk_free();
    status.wifi_signal_dbm = read_wifi_signal();
    status.uptime_s = read_uptime();
    status.power_mode = power_mode_;

    // Error code priority (same logic as jetson_probe)
    if (cpu_t > 80.0f) {
      status.error_code = 1;
      status.error_msg = "CPU over-temperature (>80°C)";
    } else if (gpu_t > 83.0f) {
      status.error_code = 2;
      status.error_msg = "GPU over-temperature (>83°C)";
    } else if (ram_total > 0 &&
      static_cast<float>(ram_used) / static_cast<float>(ram_total) > 0.90f)
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

std::string JetsonMetricsProvider::platform_name() const
{
  // Read from /proc/device-tree/model for exact board string
  std::ifstream f("/proc/device-tree/model");
  if (f.is_open()) {
    std::string model;
    std::getline(f, model);
    if (!model.empty()) {
      // Remove trailing null bytes/whitespace
      auto end = model.find('\0');
      if (end != std::string::npos) {
        model = model.substr(0, end);
      }
      return model;  // e.g., "NVIDIA Jetson Xavier NX"
    }
  }
  return "jetson_unknown";
}

std::vector<float> JetsonMetricsProvider::read_cpu_usage()
  {
    std::vector<float> usage;
    std::vector<CpuTicks> current_ticks;

    std::ifstream f("/proc/stat");
    if (!f.is_open()) { return usage; }

    std::string line;
    while (std::getline(f, line)) {
      // Parse per-core lines: "cpu0 user nice system idle ..."
      if (line.substr(0, 3) != "cpu" || line[3] == ' ') { continue; }

      CpuTicks t;
      std::istringstream iss(line);
      std::string label;
      iss >> label >> t.user >> t.nice >> t.system >> t.idle
          >> t.iowait >> t.irq >> t.softirq >> t.steal;
      current_ticks.push_back(t);
    }

    // Compute delta from previous read
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

float JetsonMetricsProvider::read_gpu_usage()
{
    std::ifstream f(gpu_load_path_);
    if (!f.is_open()) { return -1.0f; }

    int load = 0;
    f >> load;
    // Tegra sysfs reports load as 0-1000
    return static_cast<float>(load) / 10.0f;
  }

void JetsonMetricsProvider::read_temperatures(float & cpu, float & gpu, float & board)
{
    auto read_zone = [](int zone) -> float {
      std::string path = "/sys/class/thermal/thermal_zone" +
        std::to_string(zone) + "/temp";
      std::ifstream f(path);
      if (!f.is_open()) { return -1.0f; }
      int raw = 0;
      f >> raw;
      return static_cast<float>(raw) / 1000.0f;
    };

    cpu = read_zone(thermal_zone_cpu_);
    gpu = read_zone(thermal_zone_gpu_);
    board = read_zone(thermal_zone_board_);
  }

bool JetsonMetricsProvider::read_thermal_throttle()
{
    namespace fs = std::filesystem;
    try {
      for (const auto & entry : fs::directory_iterator("/sys/class/thermal/")) {
        std::string name = entry.path().filename().string();
        if (name.find("cooling_device") == std::string::npos) { continue; }

        // Check type
        std::ifstream type_f(entry.path() / "type");
        if (!type_f.is_open()) { continue; }
        std::string type;
        std::getline(type_f, type);

        bool matches = false;
        for (const auto & filter : throttle_filter_) {
          if (type.find(filter) != std::string::npos) {
            matches = true;
            break;
          }
        }
        if (!matches) { continue; }

        // Check cur_state
        std::ifstream state_f(entry.path() / "cur_state");
        if (!state_f.is_open()) { continue; }
        int state = 0;
        state_f >> state;
        if (state > 0) { return true; }
      }
    } catch (const std::exception &) {
      // Directory may not exist on non-Jetson
    }
    return false;
  }

bool JetsonMetricsProvider::read_power_throttle()
{
  // Check if EMC rate is locked (debugfs)
  std::ifstream f("/sys/kernel/debug/bpmp/debug/clk/emc/mrq_rate_locked");
  if (!f.is_open()) { return false; }
  int locked = 0;
  f >> locked;
  return locked > 0;
}

void JetsonMetricsProvider::read_memory(int32_t & used_mb, int32_t & total_mb, int32_t & swap_mb)
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

float JetsonMetricsProvider::read_disk_free()
{
  struct statvfs stat;
  if (statvfs("/", &stat) != 0) { return -1.0f; }
  return static_cast<float>(stat.f_bavail * stat.f_frsize) /
    (1024.0f * 1024.0f * 1024.0f);
}

float JetsonMetricsProvider::read_wifi_signal()
{
  std::ifstream f("/proc/net/wireless");
  if (!f.is_open()) { return 0.0f; }

  std::string line;
  // Skip 2 header lines
  std::getline(f, line);
  std::getline(f, line);

  if (std::getline(f, line)) {
    // Format: "wlan0: status link level noise ..."
    std::istringstream iss(line);
    std::string iface;
    int status_val;
    float link, level;
    iss >> iface >> status_val >> link >> level;
    return level;  // dBm (may need -256 offset on some kernels)
  }
  return 0.0f;
}

int32_t JetsonMetricsProvider::read_uptime()
{
  std::ifstream f("/proc/uptime");
  if (!f.is_open()) { return 0; }
  double uptime = 0.0;
  f >> uptime;
  return static_cast<int32_t>(uptime);
}

std::string JetsonMetricsProvider::read_power_mode()
{
  std::array<char, 128> buffer;
  std::string result;
  FILE * pipe = popen("nvpmodel -q 2>/dev/null", "r");
  if (!pipe) { return "UNKNOWN"; }

  while (fgets(buffer.data(), buffer.size(), pipe) != nullptr) {
    result += buffer.data();
  }
  pclose(pipe);

  // Parse "NV Power Mode: MODE_15W" or similar
  auto pos = result.find("NV Power Mode:");
  if (pos != std::string::npos) {
    auto start = result.find_first_not_of(" \t", pos + 14);
    auto end = result.find('\n', start);
    if (start != std::string::npos) {
      return result.substr(start, end - start);
    }
  }
  return "UNKNOWN";
}

}  // namespace rover_monitor
