## ADDED Requirements

### Requirement: CPU usage from procfs
The jetson_probe component SHALL compute per-core CPU usage by reading `/proc/stat` deltas between consecutive 0.5 Hz timer callbacks.

#### Scenario: CPU usage reported per core
- **WHEN** the 0.5 Hz timer fires
- **THEN** it SHALL report `cpu_usage_pct` as a float array with one entry per core (6 cores on Xavier NX)

### Requirement: GPU usage from sysfs
The jetson_probe SHALL read GPU load from `/sys/devices/gpu.0/load` (or known alternate Tegra sysfs paths).

#### Scenario: GPU usage reported
- **WHEN** the sysfs path is readable
- **THEN** it SHALL report `gpu_usage_pct` as a float (0-100)

#### Scenario: GPU sysfs read failure
- **WHEN** the sysfs path is not accessible
- **THEN** it SHALL report `error_code=7` and `error_msg="sysfs read failure"`

### Requirement: Temperature monitoring from thermal zones
The jetson_probe SHALL read CPU, GPU, and board temperatures from `/sys/class/thermal/thermal_zone{N}/temp`, dividing raw values by 1000 for Celsius.

#### Scenario: Temperatures nominal
- **WHEN** CPU temperature is below 80°C and GPU temperature is below 83°C
- **THEN** it SHALL report temperatures with `error_code=0`

#### Scenario: CPU over-temperature
- **WHEN** CPU temperature exceeds 80°C
- **THEN** it SHALL report `error_code=1` with `error_msg="CPU over-temperature (>80°C)"`

#### Scenario: GPU over-temperature
- **WHEN** GPU temperature exceeds 83°C
- **THEN** it SHALL report `error_code=2` with `error_msg="GPU over-temperature (>83°C)"`

### Requirement: Thermal throttle detection
The jetson_probe SHALL detect thermal throttling by reading `/sys/class/thermal/cooling_device*/cur_state` filtered for `type` matching "CPU-therm" or "GPU-therm". `cur_state > 0` indicates active throttling.

#### Scenario: Thermal throttling active
- **WHEN** any CPU-therm or GPU-therm cooling device has `cur_state > 0`
- **THEN** it SHALL report `is_thermal_throttled=true` with `error_code=5`

#### Scenario: No thermal throttling
- **WHEN** all CPU-therm and GPU-therm cooling devices have `cur_state == 0`
- **THEN** it SHALL report `is_thermal_throttled=false`

### Requirement: Power throttle detection
The jetson_probe SHALL detect power throttling by checking if SoC power rails are rate-locked below the configured nvpmodel ceiling (via `/sys/kernel/debug/bpmp/debug/clk/emc/mrq_rate_locked` or equivalent).

#### Scenario: Power throttling active
- **WHEN** EMC or SoC power rails are rate-locked
- **THEN** it SHALL report `is_power_throttled=true` with `error_code=6`

#### Scenario: No power throttling
- **WHEN** power rails are not rate-locked
- **THEN** it SHALL report `is_power_throttled=false`

### Requirement: Memory and disk monitoring
The jetson_probe SHALL read RAM from `/proc/meminfo` (MemTotal, MemAvailable) and disk free from `statvfs("/")` syscall.

#### Scenario: RAM usage high
- **WHEN** RAM usage exceeds 90%
- **THEN** it SHALL report `error_code=3` with `error_msg="RAM usage above 90%"`

#### Scenario: Disk space critical
- **WHEN** disk free drops below 1 GB
- **THEN** it SHALL report `error_code=4` with `error_msg="Disk space critical (<1 GB)"`

### Requirement: WiFi signal monitoring
The jetson_probe SHALL read WiFi signal strength from `/proc/net/wireless` (level column in dBm), without spawning a shell process.

#### Scenario: WiFi signal weak
- **WHEN** `wifi_signal_dbm` drops below -75 dBm
- **THEN** the alert engine SHALL trigger NET_DROP warning

#### Scenario: WiFi signal adequate
- **WHEN** `wifi_signal_dbm` is above -75 dBm
- **THEN** no WiFi alert SHALL be triggered

### Requirement: Power mode caching
The jetson_probe SHALL query `nvpmodel -q` once at startup and cache the result. It SHALL NOT re-read power mode every cycle.

#### Scenario: Power mode cached
- **WHEN** the jetson_probe starts
- **THEN** it SHALL run `nvpmodel -q` once, parse the power mode string (e.g., "15W", "20W"), and report it in `power_mode` for all subsequent publishes

### Requirement: Jetson status publishing at 0.5 Hz
The jetson_probe SHALL publish a `JetsonStatus` message on `/monitor/jetson` at 0.5 Hz using intra-process shared pointer hand-off.

#### Scenario: Status published every 2 seconds
- **WHEN** the 0.5 Hz timer fires
- **THEN** it SHALL publish a `JetsonStatus` message with all current metrics
