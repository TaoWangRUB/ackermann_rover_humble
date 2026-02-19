---
title: Project Context
status: Draft
owner: system_team
agent: Copilot
last_updated: 2026-02-19
doc_type: context
ros_distro: humble
---
## Overview

Ackermann autonomous rover using ROS 2 Humble.

## Reference Projects

- **Gazebo Ackermann RC Simulation** (local workspace: `gazebo_ackermann_rc_sim`)
	- Purpose: baseline Gazebo simulation and controller setup for an Ackermann RC platform.
	- Usage: informs world setup, vehicle dynamics, and control topics reused or adapted in this project.
	- Scope differences: this project adds Nav2 integration, safety watchdog, and RTAB-Map SLAM on top of the reference.

Add additional reference projects here as they are adopted (e.g., upstream Nav2 demos, RTAB-Map tutorials).
