#ifndef ROVER_MONITOR_SYSTEM_METRICS_PROVIDER_HPP
#define ROVER_MONITOR_SYSTEM_METRICS_PROVIDER_HPP

#include <memory>
#include <string>
#include "rover_monitor/msg/jetson_status.hpp"

namespace rover_monitor
{

/**
 * @brief Abstract interface for reading Jetson system metrics
 *
 * Supports both real Jetson hardware (sysfs reads) and x86_64 development machines
 * (simulated metrics). Platform detection is automatic via sentinel file check,
 * with config override option.
 *
 * JETSON IMPLEMENTATION:
 * - Reads real CPU, GPU, temperature, throttle, memory, disk, WiFi metrics from sysfs
 * - Gracefully logs and continues on missing sysfs paths
 *
 * X86_MOCK IMPLEMENTATION:
 * - REAL reads from /proc: CPU, RAM, swap, disk, uptime, WiFi (for testing alert rules)
 * - Simulated stable values: GPU=35.0%, temps=50-52°C, throttle=false (no spurious alerts)
 * - Supports ROS parameter overrides for fault injection (mock.is_thermal_throttled, etc.)
 */
class SystemMetricsProvider
{
public:
  virtual ~SystemMetricsProvider() = default;

  /**
   * @brief Read current system metrics
   * @return JetsonStatus message with current metric values
   */
  virtual rover_monitor::msg::JetsonStatus read_metrics() = 0;

  /**
   * @brief Check if running on actual Jetson hardware
   * @return true if JetsonMetricsProvider (real hardware), false if X86MockMetricsProvider
   */
  virtual bool is_jetson_hardware() const = 0;

  /**
   * @brief Get human-readable platform identifier
   * @return "jetson_xavier_nx", "jetson_orin_nx", "x86_mock", etc.
   *         Logged at probe startup for visibility (helps detect silent fallbacks)
   */
  virtual std::string platform_name() const = 0;

  /**
   * @brief Factory method with automatic platform detection
   *
   * Detection priority:
   * 1. Config override: if override != "", use that (jetson/x86_mock)
   * 2. Sentinel file: check /sys/devices/gpu.0/load exists → JetsonMetricsProvider
   * 3. Fallback: X86MockMetricsProvider
   *
   * @param override Config override ("jetson", "x86_mock", "", or any value to auto-detect)
   * @return Unique pointer to appropriate implementation
   */
  static std::unique_ptr<SystemMetricsProvider> create(const std::string& override = "");
};

}  // namespace rover_monitor

#endif  // ROVER_MONITOR_SYSTEM_METRICS_PROVIDER_HPP
