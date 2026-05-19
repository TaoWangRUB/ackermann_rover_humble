---
title: PX4 ROS 2 Interface Library
status: Draft
owner: architecture_team
agent: Copilot
last_updated: 2026-05-17
doc_type: architecture
ros_distro: humble
---

## Purpose

This project uses `px4_ros2_interface_lib` as the C++ side of the PX4 custom-mode bridge.
It is the layer that:

- registers external PX4 modes over Micro XRCE-DDS,
- receives commander arming-check requests from PX4,
- replies with `can_arm_and_run` health status for each registered mode,
- provides the C++ abstractions used by the rover mode nodes in [src/px4_bringup](../../src/px4_bringup).

In this repository, the submodule is treated as an upstream dependency plus a local patch surface.
Repository-local patches live in [patches/px4-ros2-interface-lib-arming-check-watchdog.patch](../../patches/px4-ros2-interface-lib-arming-check-watchdog.patch) and are applied with [scripts/apply_px4_ros2_patch.sh](../../scripts/apply_px4_ros2_patch.sh).

## Runtime Role

The main runtime path is:

1. PX4 commander publishes `arming_check_request` for each registered external component.
2. `px4_ros2_interface_lib` receives that request in `HealthAndArmingChecks`.
3. The active mode node fills a reply describing whether it can arm and continue running.
4. PX4 commander uses that reply to decide whether the external mode remains runnable.

The rover mode nodes in [src/px4_bringup/launch/px4_bringup.launch.py](../../src/px4_bringup/launch/px4_bringup.launch.py) depend on this mechanism to stay selectable and armed.

## MAVProxy Role On Hardware

`px4_ros2_interface_lib` itself talks to PX4 over Micro XRCE-DDS, not MAVLink. However, on real hardware this repository also relies on a host-side MAVProxy bridge for operator access and low-level debugging.

That bridge is provided by [scripts/start_px4_mavproxy_bridge.sh](../../scripts/start_px4_mavproxy_bridge.sh) and serves a different purpose from the XRCE-DDS path:

- it opens the PX4 USB CDC serial link on the host,
- exposes MAVLink on `udpin:127.0.0.1:14550`,
- lets [scripts/px4_cmd.sh](../../scripts/px4_cmd.sh) run NSH shell commands reliably,
- lets [scripts/upload_params.sh](../../scripts/upload_params.sh) set and verify parameters without repeatedly reopening the flaky USB CDC device.

So there are two parallel links on hardware:

- XRCE-DDS over the PX4 serial client and Micro XRCE Agent for mode registration, arming checks, and ROS 2 bridge traffic,
- MAVProxy over PX4 USB CDC for shell access, parameter work, and operator diagnostics.

These links are related operationally but separate at protocol level. A healthy MAVProxy shell path does not prove the XRCE-DDS request/reply path is healthy, but it does help rule out broader board, USB, or cabling failures before blaming the custom-mode layer.

## Modes In This Project

This repository uses four rover custom modes plus one offboard trajectory bridge.

| Mode | Executable | PX4 nav state | Main command mapping | Typical use |
| --- | --- | --- | --- | --- |
| `manual` | `rover_manual_mode` | `24` | throttle + steering pass-through | teleoperation and bringup |
| `speed_steering` | `rover_speed_steering_mode` | `26` | `linear.x` + normalized steering | Ackermann driving |
| `speed_attitude` | `rover_speed_attitude_mode` | `23` | `linear.x` + integrated yaw heading | heading-hold driving |
| `speed_rate` | `rover_speed_rate_mode` | `25` | `linear.x` + yaw rate | rate-based rover control |
| `trajectory` | `offboard_trajectory_mode` | offboard path | NED velocity setpoints | generic offboard control |

The launch layer supports:

- a single rover mode, for example `manual`,
- all rover modes via `mode_type:=all`,
- a comma-separated rover subset such as `manual,speed_rate`,
- the separate `trajectory` node.

Relevant entry points:

- [src/px4_bringup/launch/px4_bringup.launch.py](../../src/px4_bringup/launch/px4_bringup.launch.py)
- [scripts/start_px4_bringup_vo.sh](../../scripts/start_px4_bringup_vo.sh)
- [scripts/start_ros2_nodes.sh](../../scripts/start_ros2_nodes.sh)
- [scripts/start_jetson_session.sh](../../scripts/start_jetson_session.sh)

## Known Failure Mode

The main field issue in this project is external mode request/reply timeout on the XRCE-DDS path.

Observed symptom chain:

- commander sends `arming_check_request` every `300 ms`,
- one or more requests or replies are delayed or dropped on the serial XRCE-DDS path,
- PX4 increments `num_no_response` for that registration,
- commander eventually marks the external mode `unresponsive`,
- the matching bit flips in `failsafe_flags.mode_req_other`,
- `pre_flight_checks_pass` can flicker false,
- commander may reject the currently selected mode and fall back out of it,
- on rover this can end in `AUTO_LAND` and later `latest_disarming_reason=LANDING`.

The issue is most visible when all four rover modes are registered at once, because each request cycle must be serviced by four independent mode nodes over the same XRCE-DDS transport.

## Repository Mitigation

This repository carries a local patch for `px4_ros2_interface_lib` that reduces false unresponsive events:

- increases the ROS subscription queue depth for `ArmingCheckRequest` from `1` to `8`, matching PX4's `MAX_NUM_REGISTRATIONS`,
- keeps the mode-side watchdog alive even if PX4 temporarily clears the registration bit from `valid_registrations_mask`.

Those changes are captured in [patches/px4-ros2-interface-lib-arming-check-watchdog.patch](../../patches/px4-ros2-interface-lib-arming-check-watchdog.patch).

The practical effect is that brief request bursts no longer drop as easily on the ROS side, and temporary commander-side `unresponsive` states do not force pointless re-registration churn.

## How To Debug

### 1. Verify registration and mode announcements

Use:

```bash
./scripts/wait_px4_modes_ready.sh 4 120
docker exec jazzy_slam_x86_64 bash -lc "source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && timeout 10s ros2 topic echo /px4_modes/announce | sed -n 's/^data: //p' | sort -u"
```

Expected result:

- all requested mode nodes exist,
- mode announcements appear on `/px4_modes/announce`,
- the selected mode can later be switched to and armed.

### 2. Check commander state

Use:

```bash
docker exec jazzy_slam_x86_64 bash -lc "source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && timeout 6s ros2 topic echo --once /fmu/out/vehicle_status_v2"
docker exec jazzy_slam_x86_64 bash -lc "source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && timeout 6s ros2 topic echo --once /fmu/out/failsafe_flags"
```

Pay attention to:

- `arming_state`
- `nav_state` and `nav_state_user_intention`
- `pre_flight_checks_pass`
- `failsafe`
- `latest_disarming_reason`
- `mode_req_other`
- `local_position_invalid` and `local_velocity_invalid`

If `mode_req_other` changes by exactly one external-mode bit while local position stays valid, the failure is usually the external request/reply path rather than EKF odometry loss.

### 3. Check the transport path

Use:

```bash
./scripts/start_px4_mavproxy_bridge.sh --status
./scripts/px4_cmd.sh 'ver all'
```

This confirms that:

- the host MAVProxy bridge is healthy,
- PX4 USB shell access is alive,
- the board is responding over MAVLink/NSH before you start debugging XRCE-DDS,
- the hardware link is not failing before the ROS 2 layer even starts.

If this step fails, fix the USB/MAVLink path first. Typical symptoms are:

- ACM renumbering after reboot,
- USB CDC re-enumeration during board restart,
- a stale bridge still holding an old ACM device,
- a dead USB CDC endpoint that needs a physical replug.

Useful bridge commands:

```bash
./scripts/start_px4_mavproxy_bridge.sh
./scripts/start_px4_mavproxy_bridge.sh --status
./scripts/start_px4_mavproxy_bridge.sh --restart
./scripts/start_px4_mavproxy_bridge.sh --stop
```

If MAVProxy is healthy but the external mode still times out, the problem is further downstream on the XRCE-DDS request/reply path, not on the MAVLink shell path.

### 4. Check the local patch state

Use:

```bash
./scripts/apply_px4_ros2_patch.sh
git -C src/px4-ros2-interface-lib diff -- px4_ros2_cpp/src/components/health_and_arming_checks.cpp
```

Expected result:

- the patch script reports the watchdog patch is already applied, or applies cleanly,
- the local diff matches the patch carried under `patches/`.

### 5. Reduce concurrency when reproducing

If the failure only happens with four registered modes, reproduce with:

```bash
./scripts/start_px4_bringup_vo.sh --bridge --mode-type manual
./scripts/start_px4_bringup_vo.sh --bridge --mode-type manual,speed_rate
./scripts/start_px4_bringup_vo.sh --bridge --mode-type all
```

If single-mode and two-mode sessions are stable while `all` is not, the problem is almost always transport saturation or request burst loss rather than per-mode control logic.

## Operational Guidance

- Default to `manual` for routine bringup on hardware.
- Use `all` only when you actually need runtime mode switching across all rover modes.
- Prefer a subset such as `manual,speed_rate` when investigating commander-side timeout behavior.
- Keep the patch workflow in the parent repo instead of committing ad hoc changes directly inside the upstream submodule.

## Related Files

- [src/px4_bringup/launch/px4_bringup.launch.py](../../src/px4_bringup/launch/px4_bringup.launch.py)
- [scripts/start_px4_bringup_vo.sh](../../scripts/start_px4_bringup_vo.sh)
- [scripts/start_ros2_nodes.sh](../../scripts/start_ros2_nodes.sh)
- [scripts/start_jetson_session.sh](../../scripts/start_jetson_session.sh)
- [scripts/wait_px4_modes_ready.sh](../../scripts/wait_px4_modes_ready.sh)
- [scripts/apply_px4_ros2_patch.sh](../../scripts/apply_px4_ros2_patch.sh)
- [patches/px4-ros2-interface-lib-arming-check-watchdog.patch](../../patches/px4-ros2-interface-lib-arming-check-watchdog.patch)
- [docs/architecture/interfaces.md](../architecture/interfaces.md)