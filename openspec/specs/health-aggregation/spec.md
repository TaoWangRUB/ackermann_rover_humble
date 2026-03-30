# health-aggregation Specification

## Purpose
TBD - created by archiving change system-monitor. Update Purpose after archive.
## Requirements
### Requirement: Timer-based latest-value merge
The monitor_aggregator SHALL merge the latest available data from all three probes into a unified `RoverHealth` message on a fixed 2 Hz timer. It SHALL NOT use `ApproximateTimeSynchronizer`, which would buffer messages and introduce latency due to the 0.5 Hz jetson_probe rate.

#### Scenario: All probes reporting
- **WHEN** all three probe caches contain fresh data (within their stale timeout)
- **THEN** it SHALL publish `RoverHealth` with current data from all probes and `overall_health="OK"` (if no alerts)

#### Scenario: Probe data stale
- **WHEN** a probe's cached message age exceeds its stale timeout (cam: 500 ms, px4: 1000 ms, jetson: 4000 ms)
- **THEN** it SHALL use stale default values for that probe's fields and set `overall_health` to at least "WARN"

### Requirement: SLAM latency computation
The aggregator SHALL compute SLAM latency by comparing the `header.stamp` of the latest `/tf` transform for the `map → odom` frame against the current wall clock. It SHALL NOT use `/tf` broadcast frequency as a proxy, since `/tf` republishes stale transforms at high frequency.

#### Scenario: SLAM responsive
- **WHEN** the map→odom transform stamp delta is below 100 ms
- **THEN** it SHALL report `slam_latency_ms` with the measured delta and no SLAM alert

#### Scenario: SLAM stale
- **WHEN** the map→odom transform stamp delta exceeds 100 ms
- **THEN** it SHALL trigger the SLAM_LATE alert with severity WARN

### Requirement: Alert engine with configurable rules
The aggregator SHALL evaluate configurable alert rules against the merged data on every 2 Hz cycle. Each alert has an ID, source field, condition, severity (OK/WARN/ERROR), and message.

#### Scenario: Alert triggered
- **WHEN** a rule condition evaluates to true (e.g., `camera.frame_delta_ms > 66`)
- **THEN** it SHALL add the alert ID to `active_alerts[]` and set `overall_health` to at least the alert's severity

#### Scenario: No alerts active
- **WHEN** no rule conditions evaluate to true
- **THEN** it SHALL publish `active_alerts=[]` and `overall_health="OK"`

### Requirement: Overall health derivation
The aggregator SHALL derive `overall_health` as the worst severity across all active alerts: "OK" if none, "WARN" if any WARN, "ERROR" if any ERROR.

#### Scenario: Mixed severity alerts
- **WHEN** both WARN and ERROR alerts are active simultaneously
- **THEN** it SHALL set `overall_health="ERROR"`

### Requirement: RoverHealth publishing
The aggregator SHALL publish a `RoverHealth` message on `/monitor/health` at 2 Hz.

#### Scenario: RoverHealth published
- **WHEN** the 2 Hz timer fires
- **THEN** it SHALL publish a complete `RoverHealth` message with seq counter, all probe data, slam_latency_ms, overall_health, and active_alerts

