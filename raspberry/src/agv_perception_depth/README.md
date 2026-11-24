# agv_perception_depth

Lets Nav2 see what the LiDAR cannot: obstacles below or above its scan plane (about 13 cm above the floor). One C++ node, `depth_obstacles`, turns the Kinect v1 depth image into two small point clouds in `base_footprint`:

| Topic (namespace `/kinect`) | Type | Contents | Used by the costmaps for |
|---|---|---|---|
| `depth/image_raw`, `depth/camera_info` | in | Depth in mm (`16UC1`) and intrinsics from the [Kinect driver](https://github.com/riccardosch/kinect_v1_ros2) | |
| `depth_obstacles/points` | `sensor_msgs/PointCloud2` out | Points 5-15 cm above the floor | marking obstacles |
| `depth_obstacles/floor` | `sensor_msgs/PointCloud2` out | Points on the floor (±3 cm) | clearing: their arrival tells the costmap that the camera's field of view was refreshed |

Both are best effort, at most 10 Hz.

## How it works

[depth_to_points.cpp](src/depth_to_points.cpp), no ROS inside, tested in [test/](test/test_depth_to_points.cpp):

1. One pixel in 4 in each direction: 640×480 → 160×120 (about 19k pixels instead of 300k).
2. Each pixel with a reading between 0.5 and 1.5 m becomes a 3D point in the camera optical frame: `x = (u − cx)·z / fx`, `y = (v − cy)·z / fy`.
3. The point is moved to `base_footprint` (z = 0 on the floor) with the TF from the robot model: `base_link → kinect_link` from [agv_description](../agv_description/), `kinect_link → kinect_depth_optical_frame` from the driver.
4. Height decides: 5-15 cm → obstacle; within ±3 cm → floor; anything else is dropped.

Why not the driver's full point cloud (`point_cloud:=true`): 300k points 30 times a second in two costmaps is too much for the Pi. This node uses about 5% of one core.

## Parameters

[config/depth_obstacles.yaml](config/depth_obstacles.yaml): `step`, `rate_hz`, `min_range`, `max_range`, `min_height`, `max_height`, `floor_band`, `target_frame`.

Limits to know:
- **Closer than ~0.5 m the Kinect v1 gives no depth.** A low obstacle must be seen while approaching it; the costmap remembers it afterwards.
- **Mount tilt.** The model assumes the Kinect looks exactly level. A pitch error of 1° moves the floor by 4 cm at 2.5 m, so the floor would start to show up as obstacles far away (raise `min_height` or lower `max_range`, or measure the tilt and put it in the Xacro).
- Shiny, black or transparent surfaces and direct sunlight give no depth.

## Build and run

```bash
cd ~/agv/autonomous-mobile-robot/raspberry
source /opt/ros/jazzy/setup.bash && source ~/kinect/install/setup.bash
colcon build --packages-select agv_perception_depth
colcon test --packages-select agv_perception_depth && colcon test-result --verbose
```

Normally started by `ros2 launch agv_bringup nav.launch.py` (argument `kinect:=true`, the default). Alone, with the Kinect driver and the robot model running:

```bash
ros2 launch agv_perception_depth depth_obstacles.launch.py
```

Why only up to 15 cm and 1.5 m: the LiDAR sees everything at its own height (~13 cm) better and farther, and Kinect marks of the same walls and furniture, a few cm off beyond 1.5 m, make them thicker and close passages. The Kinect is there for what the LiDAR cannot see. `max_height` assumes the robot is ~15 cm high (to be measured).

## Status

Unit tests pass. Checked on the Pi with a synthetic depth image (the computed points match a hand calculation, and the wall appears in the Nav2 costmap), and on the robot: floor flat (pitch error ~0.1°), real objects marked. Not checked yet: thin, dark chair legs, which the robot hit during an exploration (the Kinect v1 often returns no depth on black, shiny surfaces; to be measured). History in [docs/devlog.md](../../../docs/devlog.md).
