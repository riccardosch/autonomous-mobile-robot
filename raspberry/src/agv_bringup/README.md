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
source /opt/ros/jazzy/setup.bash
source ~/kinect/install/setup.bash          # Kinect driver workspace
source ~/microros_ws/install/setup.bash     # micro_ros_agent
source ~/agv/autonomous-mobile-robot/raspberry/install/setup.bash   # last
ros2 launch agv_bringup nav.launch.py                 # LiDAR + Kinect depth
ros2 launch agv_bringup nav.launch.py kinect:=false   # LiDAR only
```

Needs `sudo apt install ros-jazzy-navigation2`. With `kinect:=true` it also starts the Kinect driver (depth only; `kinect_rgb:=true` for the colour image) and [agv_perception_depth](../agv_perception_depth/). Only one process can own the Kinect: stop `perception.launch.py` first. Do **not** run `teleop_twist_keyboard` at the same time: both publish `/cmd_vel`.

| Started | Does |
|---|---|
| `planner_server` (Smac 2D) | Path from the robot to the goal on the global costmap, kept away from obstacles; replanned every second |
| `controller_server` (Regulated Pure Pursuit) | Follows the path: `/cmd_vel`, at most 0.20 m/s and 0.8 rad/s; turns on the spot first, slows near obstacles, stops if its outline would hit an obstacle within the next 1.0 s (`collision ahead`) |
| `behavior_server` | Recoveries: back up, spin, wait |
| `bt_navigator` | The `/navigate_to_pose` action: plan → follow → recover ([behavior_trees/navigate_to_pose_agv.xml](behavior_trees/navigate_to_pose_agv.xml)) |
| `lifecycle_manager_navigation` | Configures and activates the four servers; prints `Managed nodes are active` |

Wait for `Managed nodes are active` before sending goals. At start-up `odom` exists only once the wheel ESP32 has opened its micro-ROS session (a few seconds: reset + gyro calibration); until then Nav2 and SLAM print `Timed out waiting for transform` / `queue is full`.

### Costmaps

[config/nav2.yaml](config/nav2.yaml):
- **global** (frame `map`, for planning): the SLAM map + live obstacles + inflation. Unknown space can be planned through.
- **local** (frame `odom`, 3 × 3 m around the robot, 2.5 cm cells, for following the path): live obstacles + inflation.

Live obstacles, in both, are forgotten once they are gone:
- `/scan` (ObstacleLayer): marks what the LiDAR hits, clears every cell a beam passes through.
- `/kinect/depth_obstacles/points` (Spatio-Temporal Voxel Layer, `sudo apt install ros-jazzy-spatio-temporal-voxel-layer`): obstacles 5-15 cm high, below the LiDAR plane, 0.5-1.5 m ahead. A mark fades within ~2 s when the camera looks at the place and it is empty (people, pets), otherwise after 30 s (local) or 60 s (global), so a low obstacle is remembered while the robot passes it in the Kinect's blind zone (closer than 0.5 m).

Both costmaps use the real radius, 0.17 m + 1 cm padding. Past it the cost falls off up to 0.55 m; the planner adds the cost to every step, so paths keep to the middle of passages, go around obstacles when there is room, and still take gaps with a few cm per side. On the 5 cm SLAM map a path through a very narrow gap can be one cell off centre; there the local costmap, with live sensors, decides.

### Recoveries

When the robot is stuck, in turn, up to 6 times per goal:
1. **Look again**: clear both costmaps (Kinect marks left behind the robot cannot be erased by the camera), wait 1 s for what the LiDAR sees now, back up 35 cm or, if something is behind, turn 90° on the spot, wait 1.5 s so the Kinect looks at the stuck place again from more than 0.5 m.
2. Spin 90°.
3. Clear both costmaps and wait 2 s.

### RViz

Besides the displays for mapping: **Map** `/global_costmap/costmap` and `/local_costmap/costmap` (Color Scheme = costmap), **Path** `/plan`. **2D Goal Pose** in the toolbar: click and drag on the map for position and heading.

## Status

Checked on the robot (details and dates in [docs/devlog.md](../../../docs/devlog.md)):
- `robot.launch.py`: wheel directions, LiDAR zero (`angle_offset_deg: -180.0`), left/right of the scan; maps of a room built while driving, also with slipping wheels.
- `nav.launch.py`: goals reached; boxes the LiDAR sees are avoided; the Kinect marks low obstacles (floor seen flat, no false marks).

Not yet verified on the robot: Smac 2D paths among chair legs, the current recovery sequence, the Kinect marks fading when the camera sees the place empty (checked only with synthetic images).

Known limits:
- Thin, dark obstacles below the LiDAR plane (office chair legs) can still be hit; there is no bumper yet. A hit makes the wheels slip and the SLAM can lose its place on the map.
- Wheel radius (0.035 m) and separation (0.235 m) are not checked against a tape measure; the SLAM corrects the drift.

Faults to look for if the map goes wrong:

| Symptom on the map | Cause | Fix |
|---|---|---|
| Room mirrored; map breaks at every turn | `clockwise` | `lidar_bridge.yaml` |
| Walls doubled along the driving direction | wheel radius | `odometry.yaml` + Xacro |
| Walls fan out when turning on the spot | wheel separation | `odometry.yaml` + Xacro |

**Start only one instance.** Two launches running at the same time fight over the wheel and LiDAR serial ports: the LiDAR keeps "reconnecting" and the micro-ROS session keeps being recreated. Before launching: `pgrep -af "agv_bringup|micro_ros_agent|lidar_bridge_node"` must print nothing.

`config/slam.yaml` is the Jazzy default for online asynchronous mapping; the changed lines are marked `AGV`.
