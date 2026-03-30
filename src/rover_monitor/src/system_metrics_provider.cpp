#include "rover_monitor/system_metrics_provider.hpp"
#include "rover_monitor/jetson_metrics_provider.hpp"
#include "rover_monitor/x86_mock_metrics_provider.hpp"

#include <filesystem>
#include <memory>
#include <string>

namespace rover_monitor
{

std::unique_ptr<SystemMetricsProvider> SystemMetricsProvider::create(
  const std::string & override)
{
  // Priority 1: Config override
  if (override == "jetson") {
    return std::make_unique<JetsonMetricsProvider>();
  }
  if (override == "x86_mock") {
    return std::make_unique<X86MockMetricsProvider>();
  }

  // Priority 2: Sentinel file check
  if (std::filesystem::exists("/sys/devices/gpu.0/load")) {
    return std::make_unique<JetsonMetricsProvider>();
  }

  // Priority 3: Fallback to x86_mock
  return std::make_unique<X86MockMetricsProvider>();
}

}  // namespace rover_monitor
