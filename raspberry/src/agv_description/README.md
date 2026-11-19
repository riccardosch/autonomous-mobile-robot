# agv_description

Xacro model of the robot for ROS 2 Jazzy: chassis, wheels and the frames of the IMU, the LiDAR and the Kinect v1. It is a visual and kinematic model (TF tree), not a simulation: no collisions, inertias or motor interfaces.

## Dimensions

All values are in [urdf/agv.urdf.xacro](urdf/agv.urdf.xacro) and can be overridden at launch (`name:=value`, SI units).

| Parameter | Default | Measured |
|---|---|---|
| `body_diameter` | 0.340 m | 2026-10-18 |
| `wheel_separation` | 0.235 m | 2026-10-18, approximate |
| `wheel_diameter` | 0.070 m | 2026-10-18 |
| `wheel_width` | 0.015 m | 2026-10-18 |
| `body_height` | 0.060 m | 2026-10-18, visual only |
| `body_ground_clearance` | 0.020 m | 2026-10-18, visual only |
| `imu_top_offset` | 0.015 m | IMU below the top of the body |
| `lidar_above_imu`, `lidar_x_offset` | 0.065 m, −0.115 m | 2026-10-20, scan origin |
| `kinect_above_imu`, `kinect_x_offset` | 0.040 m, +0.040 m | 2026-11-16, Kinect body centre |

Wheel radius and separation must match [agv_odometry's config](../agv_odometry/config/odometry.yaml). They have not been checked against a tape measure yet: the map from `slam_toolbox` corrects the odometry drift.

## Frames

x forward, y left, z up (REP 103). Positions relative to `base_link` with the defaults:

| Frame | Parent | Position (x, y, z) | Notes |
|---|---|---|---|
| `base_footprint` | `odom` (from [agv_odometry](../agv_odometry/)) | | Wheel axle midpoint projected on the floor |
| `base_link` | `base_footprint` | 0, 0, 0.035 | Wheel axle midpoint |
| `left_wheel_link`, `right_wheel_link` | `base_link` | 0, ±0.1175, 0 | Joints `left_wheel_joint`, `right_wheel_joint` (names used by the ESP32); axis +Y, positive = forward |
| `imu_link` | `base_link` | 0, 0, 0.030 | Axes equal to the robot axes (the firmware already maps them) |
| `laser` | `base_link` | −0.115, 0, 0.095 | Scan origin; the 180° zero offset is handled in `lidar_bridge`, not here |
| `kinect_link` | `base_link` | +0.040, 0, 0.070 | Looking along +X, no tilt |

The Kinect driver (`kinect_v1_bringup`, workspace `~/kinect`) publishes `kinect_depth_optical_frame`, `kinect_rgb_optical_frame` and `kinect_imu_frame` as children of `kinect_link`, so the camera data joins this tree without further changes.

The body is drawn centred above the axle midpoint. Not modelled yet: the caster, collisions, inertias.

## Build

```bash
sudo apt install ros-jazzy-xacro ros-jazzy-robot-state-publisher ros-jazzy-joint-state-publisher
cd ~/agv/autonomous-mobile-robot/raspberry
source /opt/ros/jazzy/setup.bash
colcon build --packages-select agv_description
source install/setup.bash
```

## Run

Normally started by [agv_bringup](../agv_bringup/). Alone:

```bash
ros2 launch agv_description description.launch.py preview:=true   # static wheels, no ESP32 needed
ros2 launch agv_description description.launch.py                 # wheels from /wheel_states
```

Run one mode at a time. Preview publishes on a separate joint-state topic and never sends motor commands.

Check a frame from the terminal: `ros2 run tf2_ros tf2_echo base_link kinect_link`.

In RViz2 on the PC: *Fixed Frame* = `base_footprint`, **RobotModel** with Description Topic `/robot_description` (Durability = Transient Local if RViz starts after the launch) and **TF** to see the axes (red x, green y, blue z).

## Status

- 2026-10-18: model and TF tree checked in RViz2.
- 2026-11-16: used in the full robot launch; the map built with `slam_toolbox` came out right with the LiDAR frame.
- 2026-11-19: `kinect_link` checked in RViz: 40 mm ahead and 70 mm above `base_link`, x forward.
