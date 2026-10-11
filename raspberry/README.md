# raspberry

ROS 2 Jazzy workspace that runs on the Raspberry Pi 5 (Ubuntu 24.04). Every node of the robot goes here, one package per folder in `src/`.

| Package | Language | Contents |
|---|---|---|
| [lidar_bridge](src/lidar_bridge/) | C++ | Reads the scans from the ESP32-C3 over USB and publishes `/scan` |

## Setup (once)

```bash
sudo apt install python3-colcon-common-extensions
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc   # optional, saves typing
```

## Build

```bash
cd raspberry
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
colcon test --packages-select lidar_bridge && colcon test-result --verbose   # unit tests
```

Every new terminal must load the workspace before using its packages:

```bash
source raspberry/install/setup.bash
```

## lidar_bridge

Reads the binary frames sent by the [LiDAR firmware](../firmware_lidar/README.md) (`esp32c3` environment) and publishes them as `sensor_msgs/LaserScan`.

```bash
ros2 launch lidar_bridge lidar.launch.py                    # uses config/lidar_bridge.yaml
ros2 launch lidar_bridge lidar.launch.py port:=/dev/ttyACM1 # different port
```

Check that it works, from a second terminal:

```bash
ros2 topic hz /scan                                         # about 8 Hz
ros2 topic echo --once /scan --no-arr                       # header and scan geometry
```

### Viewing the scan in RViz2

The Pi is headless: RViz2 runs on a PC with ROS 2 Jazzy on the same network (same `ROS_DOMAIN_ID`). Verified working on 2026-10-09.

1. On the PC: `ros2 topic hz /scan` should show about 8 Hz. If it does not, the data is not reaching the PC (firewall, network).
2. `rviz2`, then *Global Options* → **Fixed Frame** = `laser` (there is no TF tree yet).
3. *Add* → *By topic* → `/scan` → **LaserScan**.
4. In the LaserScan display, *Topic* → **Reliability Policy** = **Best Effort**. The node publishes best effort, so with the default (Reliable) RViz receives nothing.
5. Optional: *Size (m)* about 0.03-0.05 and a bright *FlatColor* to make the points easier to see.

### Topic

| Topic | Type | QoS |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | Sensor data (best effort, keep last 5) |

- 360 rays, one per degree, `angle_min = angle_offset_deg`, counter-clockwise as in ROS.
- `ranges` in metres; `inf` where the LiDAR got no echo (REP 117).
- `header.stamp` is the time of the first ray: reception time minus `scan_time`.
- `scan_time` is the measured duration of the revolution, `time_increment = scan_time / 360`.

### Parameters ([config/lidar_bridge.yaml](src/lidar_bridge/config/lidar_bridge.yaml))

| Parameter | Default | Meaning |
|---|---|---|
| `port` | `/dev/ttyACM0` | Serial port of the ESP32 |
| `frame_id` | `laser` | Frame of the scan |
| `clockwise` | `true` | The LiDAR's angles grow clockwise (YDLIDAR); the node mirrors them to the ROS convention |
| `angle_offset_deg` | `0.0` | Angle of the LiDAR's 0° in the `laser` frame |
| `range_min` | `0.12` | Minimum range in m (X2 datasheet) |
| `range_max` | `8.0` | Maximum range in m (X2 datasheet) |

`clockwise` and `angle_offset_deg` still have to be checked on the robot: put an object in front of the LiDAR and verify it appears on the +x axis of `laser`, then one on the left and verify it appears on +y.

### Behaviour

- Serial reading runs in its own thread. If the ESP32 is unplugged or reset, the node logs a warning and reopens the port every second.
- The first frame after opening the port is dropped: it can be an old one, left in the ESP32's USB buffer while nobody was reading.
- Lost frames (gaps in the sequence number) and CRC errors are logged as warnings.
- Only one program at a time can open the serial port: stop `scan_viewer.py`, `read_scan.py` or `pio device monitor` first.

### Code layout

| File | Contents |
|---|---|
| `src/frame_parser.cpp` | Frame decoding and CRC-16, no ROS dependency |
| `src/serial_port.cpp` | Minimal POSIX serial port (raw mode, poll with timeout) |
| `src/lidar_bridge_node.cpp` | ROS node: parameters, reader thread, `LaserScan` conversion |
| `test/test_frame_parser.cpp` | Unit tests: CRC check value, garbage before a frame, split reads, bad CRC |
