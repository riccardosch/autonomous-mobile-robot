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
source ~/kinect/install/setup.bash          # Kinect driver workspace, before the AGV one
source ~/agv/autonomous-mobile-robot/raspberry/install/setup.bash
ros2 launch agv_bringup nav.launch.py                 # LiDAR + Kinect depth
ros2 launch agv_bringup nav.launch.py kinect:=false   # LiDAR only
```

Needs `sudo apt install ros-jazzy-navigation2`. With `kinect:=true` it also starts the Kinect driver (depth only; `kinect_rgb:=true` for the colour image) and [agv_perception_depth](../agv_perception_depth/). Only one process can own the Kinect: stop `perception.launch.py` first. Do **not** run `teleop_twist_keyboard` at the same time: both publish `/cmd_vel`.

| Started | Does |
|---|---|
| `planner_server` (Smac 2D) | Path from the robot to the goal on the global costmap, kept away from obstacles; replanned every second |
| `controller_server` (Regulated Pure Pursuit) | Follows the path: `/cmd_vel`, at most 0.20 m/s and 0.8 rad/s; turns on the spot first, slows near obstacles, stops if its footprint would hit an obstacle within 1.5 s (`collision ahead`) |
| `behavior_server` | Recoveries when stuck: back up 35 cm, wait, spin |
| `bt_navigator` | The `/navigate_to_pose` action: plan → follow → recover |
| `lifecycle_manager_navigation` | Configures and activates the four servers; prints `Managed nodes are active` |

Costmaps ([config/nav2.yaml](config/nav2.yaml)):
- **global** (frame `map`, for planning): the SLAM map + live obstacles + inflation. Unknown space can be planned through.
- **local** (frame `odom`, 3 × 3 m around the robot, 2.5 cm cells, for following the path): live obstacles + inflation.

Live obstacles, in both, are forgotten once they are gone:
- `/scan` (ObstacleLayer): marks what the LiDAR hits, clears every cell a beam passes through.
- `/kinect/depth_obstacles/points` (Spatio-Temporal Voxel Layer, `sudo apt install ros-jazzy-spatio-temporal-voxel-layer`): obstacles 5-15 cm high, below the LiDAR plane, up to 1.5 m ahead. A mark fades within ~2 s when the camera looks at the place and it is empty (people, pets), otherwise after 30 s (local) or 60 s (global): a low obstacle is remembered while the robot passes it in the Kinect's blind zone (closer than 0.5 m).

Margins (2026-11-21): both costmaps use the real radius, 0.17 m + 1 cm padding; past it the cost falls off up to 0.55 m (`cost_scaling_factor` 4). The planner is **Smac 2D**, which adds the cell cost to every step (`cost_travel_multiplier` 2): paths keep to the middle of passages and go around obstacles when there is room. Checked on a fake map: through a 60 cm doorway the path is centred (13 cm per side), through 40 cm it is centred (3 cm per side); through 45 cm, approached diagonally, it was one 5 cm map cell off centre, and there the local costmap (2.5 cm cells, live sensors) decides. Regulated Pure Pursuit checks the next 1.0 s of motion (`collision ahead`); it slows near obstacles, so the check gets short and it can creep past them. Its acceleration limits are high on purpose (`agv_odometry` ramps the wheels): with `max_angular_accel` 2.0 it ramped from the measured turn rate and, from standstill, asked only 0.1 rad/s, too little to move the wheels, so it never started turning.

History: NavFn with an optimistic 0.15 m radius for planning brushed chair legs 1-2 cm from the robot outline, and the controller refused every such path (recorded 11 times on 2026-11-21, the obstacle confirmed by the LiDAR each time).

Recoveries ([behavior_trees/navigate_to_pose_agv.xml](behavior_trees/navigate_to_pose_agv.xml)), in turn, up to 6 per goal: **look again** (clear both costmaps, so Kinect marks left behind the robot, which the camera cannot see to erase, are gone; wait 1 s for what the LiDAR sees now; back up 35 cm slowly or, if something is behind, turn 90° on the spot; wait 1.5 s so the Kinect looks at the stuck place again from more than 0.5 m), spin 90°, clear both costmaps and wait. On 2026-11-21 the robot was stuck with old marks 1-3 cm from its outline on three sides and an object 9 cm behind: planning, spinning and backing up were all refused.

At most 0.20 m/s and 0.8 rad/s.

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
