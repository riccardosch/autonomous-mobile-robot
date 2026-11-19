# agv_bringup

Starts the whole robot with one command: wheels, LiDAR, robot model, odometry and SLAM; `nav.launch.py` adds Nav2 (see [Navigate](#navigate-nav2)).

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
| [agv_description](../agv_description/) | TF `base_footprint` → `base_link` → wheels, `imu_link`, `laser`, `kinect_link` |
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

## Navigate (Nav2)

`nav.launch.py` starts everything in `robot.launch.py` (with SLAM) plus Nav2: you give a goal on the map and the robot plans a path and drives there, around obstacles.

```bash
ros2 launch agv_bringup nav.launch.py
```

Needs `sudo apt install ros-jazzy-navigation2`. Do **not** run `teleop_twist_keyboard` at the same time: both publish `/cmd_vel`.

| Started | Does |
|---|---|
| `planner_server` (NavFn) | Path from the robot to the goal on the global costmap; replanned every second |
| `controller_server` (Regulated Pure Pursuit) | Follows the path: `/cmd_vel`, at most 0.15 m/s and 0.6 rad/s; turns on the spot first, slows near obstacles |
| `behavior_server` | Recoveries when stuck: spin, back up 30 cm, wait |
| `bt_navigator` | The `/navigate_to_pose` action: plan → follow → recover |
| `lifecycle_manager_navigation` | Configures and activates the four servers; prints `Managed nodes are active` |

Costmaps ([config/nav2.yaml](config/nav2.yaml)): the robot is a circle of radius 0.17 m, obstacles are inflated by 0.35 m.
- **global** (frame `map`): the SLAM map + obstacles from `/scan` + inflation. Unknown space can be planned through.
- **local** (frame `odom`, 3 × 3 m around the robot): obstacles from `/scan` + inflation.

Obstacles are seen only in the LiDAR plane, about 13 cm above the floor: lower or overhanging ones are invisible until the Kinect depth is added.

In RViz on the PC, besides the displays for mapping: **Map** `/global_costmap/costmap` and `/local_costmap/costmap` (Color Scheme = costmap), **Path** `/plan`. Then **2D Goal Pose** in the toolbar: click and drag on the map for position and heading.

## Status

Run on the robot on 2026-11-16: the whole launch works and builds a map of a room. The map closed correctly even when the wheels slipped on a carpet, because `slam_toolbox` corrects the odometry with the scans.

Checked on the robot:
- Wheel directions: `i` moves the robot forward, `j` turns it left.
- LiDAR zero: a box in front of the robot showed up behind it, so `lidar_bridge.yaml` now has `angle_offset_deg: -180.0`. After the change the map came out right.

Nav2 run on the robot on 2026-11-20 (`nav.launch.py`):
- A goal about 1 m ahead on a free floor: reached.
- A box taller than the scan plane on the path: `/plan` went around it and the robot followed.
- A box on the robot's left showed up on the left (+y) in the costmap: the scan is not mirrored.
- An obstacle **lower than the LiDAR** was hit: the wheels kept turning, the odometry counted motion that did not happen, the SLAM lost its place on the map and every later goal failed. Restarting the launch (new map) was the way out. The Kinect depth is needed in the costmap.
- At start-up `odom` exists only once the wheel ESP32 has opened its micro-ROS session (a few seconds: reset + gyro calibration). Until then Nav2 and SLAM print `Timed out waiting for transform` / `queue is full`; wait for `Managed nodes are active` before sending goals.

Not done yet (postponed on 2026-11-19, not blocking while the map is right): a check of wheel radius (0.035 m) and separation (0.235 m) against a tape measure (1 m straight, one full turn on the spot).

The Kinect is not started by this launch: its frame `kinect_link` is in the model, and the driver ([kinect_v1_perception](../kinect_v1_perception/)) attaches the camera frames to it.

Faults to look for if the map goes wrong:

| Symptom on the map | Cause | Fix |
|---|---|---|
| Room mirrored; map breaks at every turn | `clockwise` | `lidar_bridge.yaml` |
| Walls doubled along the driving direction | wheel radius | `odometry.yaml` + Xacro |
| Walls fan out when turning on the spot | wheel separation | `odometry.yaml` + Xacro |

**Start only one instance.** Two `robot.launch.py` running at the same time fight over the wheel and LiDAR serial ports: the LiDAR keeps "reconnecting" and the micro-ROS session keeps being recreated. Before launching: `pgrep -af "agv_bringup|micro_ros_agent|lidar_bridge_node"` must print nothing.

`config/slam.yaml` is the Jazzy default for online asynchronous mapping; the changed lines are marked `AGV`.
