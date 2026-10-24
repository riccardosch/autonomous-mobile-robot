# agv_description

Custom Xacro model for ROS 2 Jazzy, built from measurements on
2026-10-18 (interpreted as centimetres and converted to SI units).

| Parameter | Default | Status |
|---|---|---|
| `body_diameter` | 0.340 m |
| `wheel_separation` | 0.235 m |
| `wheel_diameter` | 0.070 m | 
| `wheel_width` | 0.015 m | 
| `body_height` | 0.080 m | Provisional, visual only |
| `body_ground_clearance` | 0.015 m | Provisional, visual only |

Dimensions are defined in `urdf/agv.urdf.xacro` and can be overridden at launch.
This is a visual and kinematic model, not a dynamics simulation or a motor
controller. Body collision geometry and inertias await measurements.

## Frames

- `base_footprint`: ground projection of the wheel axle midpoint.
- `base_link`: wheel axle midpoint, 35 mm above the floor at the default radius.
- `left_wheel_link`, `right_wheel_link`: centres at y = +/-117.5 mm.
- x is forward, y left, z up. Both wheel axes are +Y, so positive rotation
  corresponds to forward rolling. Joint names match the ESP32:
  `left_wheel_joint` and `right_wheel_joint`.

The visual body is assumed centred above the axle midpoint; check this on the
chassis. No IMU, LiDAR or caster transforms are invented: these will be added
once mounting positions are measured. In particular, this package does not yet
connect `imu_link` or `laser` to the robot. There is no `odom` transform yet.

## Install and build on the Pi

```bash
sudo apt install ros-jazzy-xacro ros-jazzy-robot-state-publisher ros-jazzy-joint-state-publisher
cd ~/agv/autonomous-mobile-robot/raspberry
source /opt/ros/jazzy/setup.bash
colcon build --packages-select agv_description
source install/setup.bash
```

Preview without the ESP32 or agent (publishes stationary wheels):

```bash
ros2 launch agv_description description.launch.py preview:=true
```

Or, with the agent running and the ESP32 publishing `/wheel_states`:

```bash
ros2 launch agv_description description.launch.py
```

Run only one mode at a time. Preview uses a separate joint-state topic and
never sends motor commands. Live mode remaps robot_state_publisher's input to
`/wheel_states`; wheel TF becomes available when feedback arrives.

On the PC, open RViz with the same ROS domain and network setup as the LiDAR.
Set Fixed Frame to `base_footprint`, add RobotModel using `/robot_description`,
and add TF to inspect the axes. Set the RobotModel description subscription
to Transient Local if the model does not appear after starting RViz late.

Example override after measuring the body height:

```bash
ros2 launch agv_description description.launch.py preview:=true body_height:=0.090
```

## Next measurements and integration

Measure body height, ground clearance, axle offset from the body centre, and
sensor positions relative to `base_link`. For the LiDAR use the scan origin;
for the IMU account for the firmware's existing axis mapping. Then add sensor
TF and the ros2_control interface. The controller must use the same wheel
radius and separation, with feedback timeout and stop-on-deactivation handling.
