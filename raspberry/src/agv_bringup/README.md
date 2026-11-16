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

Run on the robot on 2026-11-16: the whole launch works and builds a map of a room. The map closed correctly even when the wheels slipped on a carpet, because `slam_toolbox` corrects the odometry with the scans.

Checked on the robot:
- Wheel directions: `i` moves the robot forward, `j` turns it left.
- LiDAR zero: a box in front of the robot showed up behind it, so `lidar_bridge.yaml` now has `angle_offset_deg: -180.0`. After the change the map came out right.

Not done yet:
- Wheel radius (0.035 m) and separation (0.235 m) are measured values and the odometry looks right in RViz. A check against a tape measure (1 m straight, one full turn on the spot) has not been recorded.
- Left/right mirroring of the scan was judged only from the shape of the map. A box on the robot's left must show up along the green (y) axis.

Faults to look for if the map goes wrong:

| Symptom on the map | Cause | Fix |
|---|---|---|
| Room mirrored; map breaks at every turn | `clockwise` | `lidar_bridge.yaml` |
| Walls doubled along the driving direction | wheel radius | `odometry.yaml` + Xacro |
| Walls fan out when turning on the spot | wheel separation | `odometry.yaml` + Xacro |

**Start only one instance.** Two `robot.launch.py` running at the same time fight over the wheel and LiDAR serial ports: the LiDAR keeps "reconnecting" and the micro-ROS session keeps being recreated. Before launching: `pgrep -af "agv_bringup|micro_ros_agent|lidar_bridge_node"` must print nothing.

`config/slam.yaml` is the Jazzy default for online asynchronous mapping; the changed lines are marked `AGV`.
