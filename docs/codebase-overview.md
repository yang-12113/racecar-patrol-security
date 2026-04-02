# Codebase Overview

## Project Goal

This repository is the code mainline for the patrol-security racecar project:

- patrol on a fixed map with Nav2
- detect people with YOLO on Ascend/ACL
- recognize known faces vs `unknown`
- interrupt patrol to track an intruder
- return to the nearest patrol waypoint after the alert clears

## Main Code Areas

### `face_tools/`

This is the high-value application layer on top of the base car stack.

- `patrol_supervisor.py`
  - top-level state machine
  - drives `PATROL -> ALERT_TRACK -> RETURN_TO_ROUTE -> SAFE_STOP`
  - sends Nav2 goals, watches follow state, and decides when to cancel or resume patrol
- `yolo_face_track_alarm.py`
  - camera inference pipeline
  - runs person detection, face association, identity decision, and writes `/tmp/face_follow_state.json`
- `face_follow_controller.py`
  - reads the follow-state file and publishes `/car_cmd_vel`
  - includes obstacle-aware speed/steering adjustment from `/scan`
- `patrol_preflight_check.py`
  - validates map, waypoints, transforms, and nav readiness before a patrol demo starts
- `wait_localization_ready.py`
  - waits for localization to converge and can republish initial pose
- `publish_initial_pose.py`
  - helper for AMCL/Nav2 startup alignment
- `capture_patrol_waypoint.py`
  - captures patrol waypoints for demo routes
- `sync_face_assets_to_car.py`
  - sync helper for models, scripts, and face assets
- `run_*.sh`
  - full-stack wrappers for follow demos, patrol demos, and navigation bringup

### `home/racecar/src/racecar/`

This is the base ROS2 package for the car.

- `launch/`
  - navigation, sensor, AMCL, and car startup entrypoints
- `config/`
  - Nav2, EKF, lidar filter, camera, and planner parameters
- `scripts/navigation_test.py`
  - sequential Nav2 waypoint test tool
- `src/car_controller_new.cpp`
  - main vehicle controller publishing `/car_cmd_vel`
  - subscribes to plan, odometry, goal, and light/stop topics
- `src/laser_2d_new.cpp`, `src/scan_node.cpp`, `src/car_control_all.cpp`
  - base motion and sensor-side control components
- `map/` and `maps/`
  - saved maps used by navigation and demos

### Other ROS2 Packages Under `home/racecar/src/`

- `hipnuc_imu/`
  - IMU integration and launch helpers
- `rf2o_laser_odometry/`
  - laser odometry package
- `slam_gmapping/`
  - mapping package
- `robot_navigation2/`
  - additional Nav2 bringup assets and params

## Runtime Chain

The current project is split into two main layers:

1. Base car and navigation layer
   - lidar, IMU, odometry, localization, Nav2, and low-level controller
2. Security behavior layer
   - YOLO + face recognition + state machine + intruder follow behavior

Typical alert-patrol demo flow:

1. bring up ROS2, sensors, localization, and Nav2
2. run preflight checks
3. run `patrol_supervisor.py`
4. supervisor dispatches waypoint goals
5. `yolo_face_track_alarm.py` writes a live follow-state file when an intruder is detected
6. supervisor cancels patrol and enters `ALERT_TRACK`
7. `face_follow_controller.py` publishes follow commands to `/car_cmd_vel`
8. once the target is lost or timeout/distance limits trigger, supervisor returns to the nearest waypoint

## Recommended Working Method

Use the three-layer split below going forward:

### GitHub private repo

Store:

- source code
- launch scripts
- configs
- docs
- recovery notes

Do not store:

- full-system tar backups
- large models
- face databases
- bag files
- generated build/install/log directories

### USB drive

Store:

- full-system backup sets like `car_backup_20260402_0900`
- large models
- face databases
- maps, recordings, bag files, and release bundles

### Car

Use the car as the runtime target, not the source of truth.

Recommended loop:

1. edit code in this repository on the PC
2. commit stable milestones to GitHub
3. keep large assets and backups on the USB drive
4. sync only the tested code/assets back to the car for runtime validation

## Current Mainline Scope

The first import intentionally focuses on the code and docs most valuable for development:

- `face_tools/`
- `home/racecar/`
- `home/racecar/src/`
- `docs/`

Large runtime assets and raw full-system backups remain outside Git by design.
