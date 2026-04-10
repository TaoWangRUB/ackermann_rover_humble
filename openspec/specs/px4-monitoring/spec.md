# px4-monitoring Specification

## Purpose
TBD - created by archiving change system-monitor. Update Purpose after archive.
## Requirements
### Requirement: PX4 heartbeat monitoring via uORB timestamp
The px4_probe component SHALL use the `timestamp` field of `/fmu/out/vehicle_global_position` as a liveness signal, since PX4 does not publish a standalone heartbeat topic over XRCE-DDS.

#### Scenario: PX4 heartbeat healthy
- **WHEN** the timestamp delta between the latest uORB message and current time is below 1000 ms
- **THEN** it SHALL report `connected=true` with `heartbeat_age_ms` set to the delta and `error_code=0`

#### Scenario: PX4 heartbeat lost
- **WHEN** the timestamp delta exceeds 1000 ms
- **THEN** it SHALL report `connected=false` with `error_code=1` and `error_msg="PX4 uORB heartbeat lost"`

### Requirement: Battery state monitoring
The px4_probe SHALL subscribe to `/fmu/out/battery_status` and report voltage, current, and remaining percentage.

#### Scenario: Battery nominal
- **WHEN** `battery_remaining_pct` is above 40%
- **THEN** it SHALL report battery fields with `error_code=0`

#### Scenario: Battery low
- **WHEN** `battery_remaining_pct` drops below 40%
- **THEN** it SHALL report `error_code=3` with `error_msg="Battery low"`

#### Scenario: Battery critical
- **WHEN** `battery_remaining_pct` drops below 20%
- **THEN** it SHALL report `error_code=2` with `error_msg="Battery critical — return to base"`

### Requirement: Vehicle state tracking
The px4_probe SHALL subscribe to `/fmu/out/vehicle_status` and report arming state and navigation state.

#### Scenario: Vehicle status received
- **WHEN** a vehicle_status message is received
- **THEN** it SHALL report `armed` (bool), `nav_state` (int), and `nav_state_label` (human-readable string, e.g., "OFFBOARD", "POSCTL")

### Requirement: XRCE-DDS agent disconnect detection
The px4_probe SHALL detect when the Micro XRCE-DDS agent becomes unresponsive by monitoring subscription activity across all PX4 topics.

#### Scenario: XRCE-DDS agent disconnected
- **WHEN** no messages arrive on any subscribed PX4 topic for more than 2000 ms
- **THEN** it SHALL report `error_code=4` with `error_msg="XRCE-DDS agent disconnect"`

### Requirement: PX4 status publishing
The px4_probe SHALL publish a `Px4Status` message on `/monitor/px4` using intra-process shared pointer hand-off.

#### Scenario: Status published periodically
- **WHEN** any PX4 uORB subscription callback fires
- **THEN** it SHALL publish an updated `Px4Status` message via `std::unique_ptr<Px4Status>` publish API

