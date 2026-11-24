# agv_map3d

3D map of the house (mission mode 2) with [RTAB-Map](https://github.com/introlab/rtabmap_ros): the Kinect v1 gives colour and depth, the LiDAR and the 2D SLAM give the robot pose. The result is a database with every view, from which the coloured 3D point cloud is exported.

## Run (manual drive)

Terminal 1, the robot with the 2D SLAM:
```bash
ros2 launch agv_bringup robot.launch.py
```
Terminal 2, the 3D mapper (it also starts the Kinect driver):
```bash
ros2 launch agv_map3d rtabmap.launch.py name:=casa3d
```
Terminal 3, drive slowly, facing walls and furniture from about 1 m:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.1 -p turn:=0.4
```
Ctrl+C in terminal 2 closes the database `~/maps/casa3d.db`.

| Argument | Default | |
|---|---|---|
| `name` | `map3d` | database `~/maps/<name>.db` |
| `keep` | `false` | `true`: continue that database instead of replacing it |
| `driver` | `true` | `false` if the Kinect driver already runs in `registered` mode |

Only one process can own the Kinect: do not run `nav.launch.py` with `kinect:=true` or `kinect_v1_perception` at the same time.

## How it works

| Node (namespace `/rtabmap`) | Does |
|---|---|
| Kinect driver | `depth_mode: registered`: the depth image aligned pixel by pixel to the colour image (`/kinect/depth_registered/image_raw`, `/kinect/rgb/image_raw`) |
| `rgbd_sync` | pairs colour, depth and camera_info taken at the same moment into one RGB-D message |
| `rtabmap` | once a second, if the robot moved 10 cm or turned 6°, stores a view (colour, depth, scan, pose); finds places seen before from image features and corrects the graph (loop closure) |

Frames: the robot pose comes from the 2D SLAM (TF `map → base_footprint`, already corrected), so RTAB-Map uses `map` as its odometry frame and publishes only `map3d → map`. The 3D model is aligned with the 2D map and `map → odom` keeps a single owner (`slam_toolbox`).

Parameters: [config/rtabmap.yaml](config/rtabmap.yaml), with the values that differ from RTAB-Map's defaults commented.

## Look at the result

In RViz on the PC: *Fixed Frame* `map3d`, **PointCloud2** `/rtabmap/cloud_map` (the 3D map grows while driving), **Image** `/kinect/rgb/image_raw` (Reliability Best Effort).

Afterwards, a coloured point cloud file for any 3D viewer (MeshLab, CloudCompare):
```bash
rtabmap-export --cloud --output_dir ~/maps ~/maps/casa3d.db      # -> ~/maps/casa3d_cloud.ply
rtabmap-info ~/maps/casa3d.db                                    # views, links, loop closures
```
`rtabmap-databaseViewer ~/maps/casa3d.db` (a window application: on the PC, with RTAB-Map installed) shows every view and the graph.

## Status

- Without the robot (synthetic colour, depth, scan and motion): one view a second, ~40 ms each.
- On the robot, driven by hand around a room for 9 min (4.6 m): 501 views, 20 loop closures, ~70-80 ms per view, low CPU. The exported cloud (445k points) has straight, single walls and a consistent room shape.
- Colours were almost black: the room was dimly lit at night and the Kinect v1 colour camera needs light (the depth uses its own infrared light and works in the dark). To redo with the room lit.
- Not checked yet: the floor height in the cloud.
