# agv_bringup

Starts the whole robot with one command: wheels, LiDAR, robot model, odometry and SLAM.

```bash
source /opt/ros/jazzy/setup.bash
source ~/microros_ws/install/setup.bash                               # micro_ros_agent
source ~/agv/autonomous-mobile-robot/raspberry/install/setup.bash
ros2 launch agv_bringup robot.launch.py
```

Keep the robot still while it starts: opening the serial port resets the wheel ESP32, which calibrates the gyro.

| Argument | Default | |
|---|---|---|
| `slam` | `true` | Build a map with `slam_toolbox`; `false` to test odometry and the LiDAR alone |
| `wheels_port` | `/dev/ttyUSB0` | Wheel ESP32 (micro-ROS, `wheels_baud` 921600) |
| `lidar_port` | `/dev/ttyACM0` | LiDAR ESP32-C3 |

| Started | Gives |
|---|---|
| `micro_ros_agent` | `/wheel_states`, `/imu/data_raw`, `/wheel_cmd` (wheel ESP32 with the `microros` firmware) |
| [lidar_bridge](../lidar_bridge/) | `/scan` |
| [agv_description](../agv_description/) | TF `base_footprint` → `base_link` → wheels, `imu_link`, `laser` |
| [agv_odometry](../agv_odometry/) | `/odom`, TF `odom` → `base_footprint`; `/cmd_vel` → `/wheel_cmd` |
| `slam_toolbox` ([config/slam.yaml](config/slam.yaml)) | `/map`, TF `map` → `odom` |

Needs `sudo apt install ros-jazzy-slam-toolbox ros-jazzy-nav2-map-server`.

## Drive and map

In a second terminal on the Pi (the keys are read from this terminal):

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.1 -p turn:=0.5
```

On the PC, `rviz2` with *Fixed Frame* = `map`, and the displays **Map** (`/map`, Durability = Transient Local), **LaserScan** (`/scan`, Reliability = Best Effort), **TF** and **RobotModel** (Description Topic `/robot_description`).

Drive slowly, turn gently and come back over places already seen. Save the map:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/casa        # writes casa.pgm and casa.yaml
```

## Status

Written on 2026-11-14 and **not run on the robot yet**. Checked: the launch file parses, `slam_toolbox` configures and activates with `config/slam.yaml`. The parts below were never verified on the robot, so the first maps may be wrong; this is how each fault looks:

| Not verified yet | Symptom on the map | Fix |
|---|---|---|
| LiDAR angle direction (`clockwise`) | Room mirrored; map breaks at every turn | `lidar_bridge.yaml` |
| LiDAR zero angle (`angle_offset_deg`) | Scan rotated with respect to the robot model; walls smear when driving straight | `lidar_bridge.yaml` |
| Wheel radius | Walls doubled along the driving direction | `odometry.yaml` + Xacro |
| Wheel separation | Walls fan out when turning in place | `odometry.yaml` + Xacro |
| Wheel direction / left-right swap | The robot model moves backwards or turns the wrong way in RViz | firmware or `odometry.yaml` |

`config/slam.yaml` is the Jazzy default for online asynchronous mapping; the changed lines are marked `AGV`.
