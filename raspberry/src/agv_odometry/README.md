# agv_odometry

Wheel odometry for the two-wheel (differential-drive) robot, and the opposite direction: robot velocity commands to wheel speed targets. One Python node, `odometry`, between the [wheel ESP32](../../../firmware_wheels_imu/README.md#micro-ros-node) and the rest of ROS 2.

| Topic | Direction | Type | Contents |
|---|---|---|---|
| `/wheel_states` | in | `sensor_msgs/JointState` | Wheel positions (rad) and speeds (rad/s) from the ESP32, 50 Hz |
| `/cmd_vel` | in | `geometry_msgs/Twist` | Robot speed: `linear.x` (m/s, forward), `angular.z` (rad/s, anticlockwise) |
| `/odom` | out | `nav_msgs/Odometry` | Position and heading of `base_footprint` in `odom`, and the robot speed |
| `/wheel_cmd` | out | `sensor_msgs/JointState` | Wheel speed targets (rad/s) to the ESP32, 20 Hz while `/cmd_vel` is alive |
| TF | out | | `odom` → `base_footprint` |

This is a first, readable version to understand the odometry. `diff_drive_controller` (ros2_control) will later do the same job in the standard way.

## How it works

**Odometry.** For every `/wheel_states` message the node takes how far each wheel turned since the previous one (Δθ, rad) and, with the wheel radius *r* and the distance between the wheels *b*:

```
dl = r · Δθ_left     dr = r · Δθ_right       distance rolled by each wheel (m)
d     = (dr + dl) / 2                        forward motion of the robot centre
Δyaw  = (dr − dl) / b                        rotation of the robot
x += d · cos(yaw + Δyaw/2)    y += d · sin(yaw + Δyaw/2)    yaw += Δyaw
```

If the right wheel rolls less than the left, Δyaw is negative and the odometry knows the robot turned right. The position is the sum of all these small steps, so its error grows with the distance: that is why the IMU and later the LiDAR (SLAM) correct it.

- The positions come from the encoders, so the odometry measures what the wheels really did, not what was asked.
- If the wheel positions jump by more than `max_wheel_step_rad` between two messages, the ESP32 has restarted (its positions start again from 0): that step is skipped.
- **Single-channel encoders:** the ESP32 takes the direction of each tick from the motor command. If the robot is pushed by hand, the odometry counts in the direction of the last command.
- The time stamps are the ESP32's, synchronised with the Pi clock.

**Velocity commands.** For a robot speed *v* and turn rate *ω*: `left = (v − ω·b/2) / r`, `right = (v + ω·b/2) / r`. The node sends these targets at 20 Hz while `/cmd_vel` keeps arriving, ramping them at most `max_wheel_accel` (20 rad/s², 0 to 4 rad/s in 0.2 s), so the wheels never get a step from standstill (see test T9 in the [wheel firmware test record](../../../firmware_wheels_imu/tests/README.md)). If `/cmd_vel` stops for `cmd_timeout_s` (0.5 s) the targets ramp to 0 and the node stops sending; the ESP32 watchdog stops the wheels 0.5 s later in any case.

## Parameters

[config/odometry.yaml](config/odometry.yaml). The wheel radius and separation must match the robot model ([agv.urdf.xacro](../agv_description/urdf/agv.urdf.xacro)); both are approximate until the odometry calibration.

| Parameter | Default | Meaning |
|---|---|---|
| `wheel_radius` | 0.035 m | Wheel radius |
| `wheel_separation` | 0.235 m | Distance between the centres of the two wheel treads |
| `left_joint`, `right_joint` | `left_wheel_joint`, `right_wheel_joint` | Joint names in `/wheel_states` and `/wheel_cmd` |
| `odom_frame`, `base_frame` | `odom`, `base_footprint` | Frames of `/odom` and of the TF |
| `publish_tf` | true | Publish `odom` → `base_footprint`; off when an EKF publishes it |
| `max_wheel_step_rad` | 1.0 rad | Larger jumps between two messages are taken as an ESP32 restart |
| `cmd_rate_hz` | 20 Hz | `/wheel_cmd` rate |
| `cmd_timeout_s` | 0.5 s | `/cmd_vel` older than this counts as stopped |
| `max_wheel_accel` | 20 rad/s² | Ramp limit on the wheel targets |

The odometry covariances are not set yet (all zero): they will be chosen with the EKF (`robot_localization`).

## Run

Needs the wheel ESP32 with the `microros` build and the agent running (see the [firmware README](../../../firmware_wheels_imu/README.md#micro-ros-node)).

```bash
cd ~/agv/autonomous-mobile-robot/raspberry
source /opt/ros/jazzy/setup.bash
colcon build --packages-select agv_description agv_odometry
source install/setup.bash
ros2 launch agv_odometry odometry.launch.py     # robot model (live wheels) + odometry
```

Drive it with the keyboard, from another terminal (wheels on the floor, space around the robot):

```bash
source /opt/ros/jazzy/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.1 -p turn:=0.5
```

Watch the position:

```bash
ros2 topic echo /odom --field pose.pose.position
```

In RViz2 on the PC: *Fixed Frame* `odom`, add **RobotModel** (`/robot_description`) and **TF**; the model moves as the robot drives.

## Code layout

| File | Contents |
|---|---|
| [agv_odometry/odometry_node.py](agv_odometry/odometry_node.py) | The node: odometry, `/cmd_vel` to wheel targets with the ramp |
| [config/odometry.yaml](config/odometry.yaml) | Parameters |
| [launch/odometry.launch.py](launch/odometry.launch.py) | `agv_description` (live wheel states) plus the node |
| [package.xml](package.xml), [setup.py](setup.py), [setup.cfg](setup.cfg) | Package metadata, install rules |
