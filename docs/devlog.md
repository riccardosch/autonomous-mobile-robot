# Development log

Newest entries go at the top. Each entry records what was done, what was learned, the problems found and the next steps.
The *current* state of each part lives in the READMEs; this file keeps the history of how it got there.

---

## 2026-11-24 / 2026-11-25: First 3D map (block 4a)

### Done
- New package `agv_map3d`: `rtabmap.launch.py` starts the Kinect driver in `registered` mode (depth aligned to colour), `rgbd_sync` and RTAB-Map in the `/rtabmap` namespace; database `~/maps/<name>.db`.
- RTAB-Map takes the robot pose from the 2D SLAM (`odom_frame_id: map`) and publishes only `map3d → map`, so `slam_toolbox` stays the only owner of `map → odom` and the 3D model is aligned with the 2D map.
- Dry run with synthetic RGB-D data, then a 9-minute hand-driven run in a room: 501 views, 20 loop closures (4 global, 16 local), ~70-80 ms per view. `rtabmap-export` gave a 445k-point cloud with straight, single walls.

### Learned
- In the dark the Kinect v1 colour image is almost black (median brightness 2/255) while the depth is fine: it has its own infrared projector. 3D colours need a lit room.
- The fake data for a dry run must carry real timestamps on the TF too; with stamp 0 RTAB-Map waited forever (`would require extrapolation`).

### Next steps
- Redo a short run with the room lit; check the floor height in the cloud.
- Block 4b: the `coverage` node (route along walls and furniture on the saved 2D map) and `map3d.launch.py` with `slam_toolbox` in localization mode.

---

## 2026-11-24: Explorer cleanup (no change in behaviour)

### Done
- `agv_explore` split by responsibility. The node (`explorer_node.cpp`) only talks to ROS and runs an explicit state machine (`WaitingForData → Idle ⇄ Driving → Finished`, transitions in one place). The decisions moved to ROS-free modules: `frontiers` (find), `goal_selector` (score, order, goal spot), `frontier_memory` (one class for failures and retries, blacklist, up-close approaches and arrivals, with time passed in), `map_file` (`.pgm`/`.yaml`).
- Two numbers that were in the code are parameters now: `seen_on_way_after_s` (3 s) and `standoff_search_dist` (0.4 m). The parameter file key is `/**:`.
- Unit tests from 8 to 20: the memory, the goal choice and the map file are now tested without ROS. The simulation gives the same result as before the split (17 goals, 100% of the house, nothing blacklisted; with two forced failures, two retries then through).
- READMEs of `agv_explore`, `agv_bringup`, `agv_perception_depth` and the comments in `nav2.yaml` rewritten to describe the current state only; values that had gone stale were fixed (goal cost 85, not 60; `collision ahead` horizon 1.0 s, not 1.5 s; `nav.launch.py` does start the Kinect). The history stays in this log.

### Learned
- Fixing on the robot one symptom at a time had left five lists with overlapping meanings and state changes spread over four functions; each fix risked breaking another. Writing the rules down as one tested class showed them in a few lines.

---

## 2026-11-20 / 2026-11-21: Kinect depth in the costmaps, autonomous exploration

### Done
- `agv_perception_depth`: C++ node `depth_obstacles`, Kinect depth image → one pixel in 4 → points in `base_footprint` → obstacles (5-15 cm high, 0.5-1.5 m ahead) and floor clouds. About 5% of one core. Checked live: floor flat (pitch error ~0.1°), real objects marked.
- Costmaps: LiDAR in an ObstacleLayer, Kinect in a Spatio-Temporal Voxel Layer (marks fade within ~2 s when the camera sees the place empty, otherwise after 30 s local / 60 s global). Local costmap with 2.5 cm cells and the real radius (0.17 m + 1 cm); global costmap on the SLAM map with an optimistic radius (0.15 m), so passages with a few cm per side are proposed and the local costmap decides.
- Own behavior tree: when stuck, clear the local costmap, back up 35 cm, wait 2 s (the Kinect sees the place face on from more than 0.5 m), plan again; then clear all and wait; then spin.
- `agv_explore`: C++ node `explorer`, frontier exploration with no user goals. Frontiers on the SLAM map, best one close and ahead of the robot, goal 0.8 m in front of the frontier facing it (safe spot on the global costmap), goals replaced without stopping, blacklist for unreachable or unseeable frontiers, map saved to `~/maps` at the end (`.pgm`/`.yaml` and `slam_toolbox` pose graph).
- Speed 0.2 m/s and 0.8 rad/s.
- Tested without the robot: synthetic depth images through the whole chain into the costmap; a simulated house (fake Nav2, map revealed by ray casting) for the explorer.
- Robot run `prova3` (2026-11-21): the room explored in 2.0 min, no frontier blacklisted, one stop solved by the back-up-and-look recovery, map saved. Remaining frontiers all under 25 cm.

### Learned
- A low obstacle under the LiDAR plane makes the wheels slip; the odometry, then the SLAM, lose the robot's place on the map. The Kinect has to cover what the LiDAR cannot see.
- With a 2D voxel/obstacle layer the LiDAR rays clear the Kinect's marks on a low box; with a persistent 3D layer the Kinect's marks never go away and fill the costmap (63% of the free space was forbidden in the first exploration). Marks must fade, but not while the obstacle is in the Kinect's blind zone (< 0.5 m).
- The Kinect only adds value below the LiDAR plane and close by: at 2-2.5 m its few-cm errors thickened walls and closed passages.
- `collision ahead` from Regulated Pure Pursuit is about the next 1.5 s of the path, not the current position. Spinning near obstacles fails too; backing up and looking again works.
- The planner needs the goal on a spot where the robot fits: a frontier cell is usually against a wall.
- A launch argument like `5` becomes an integer; a node parameter declared as double then aborts the node at start.
- A parameter file key must match the node's full name: `depth_obstacles:` does not match `/kinect/depth_obstacles`, and the whole file was silently ignored. Until this was found (after `prova3`), every robot run had the Kinect at the old code defaults (2.5 m, 25 cm), which explains phantom marks a few cm in front of walls. Now `/**:` and code defaults equal to the file.
- Simulating the world around the real node found a race (arrival reported before the map update) that would have been hard to see on the robot.
- One failed plan is not proof that a place is unreachable: an open door was blacklisted at the first `Failed to create plan` and the exploration ended. Frontiers are now retried up to 3 times, 20 s apart.
- `slam_toolbox/save_map` starts `map_saver`, which gives up if `/map` does not arrive within 2 s; the explorer now writes the map files itself.
- `collision ahead` was right: a read-only recorder on the robot (Regulated Pure Pursuit's `/lookahead_collision_arc` against the local costmap) found the outline 1-2 cm into cells of chair legs that the LiDAR confirmed. The path was wrong, not the check: NavFn with an optimistic planning radius hugged obstacles. Now Smac 2D with the real radius; on a fake doorway the path is centred.
- A robot can get boxed in by memories: Kinect marks behind it (the camera cannot look there to erase them) and a slightly blurred SLAM map put cells 1-3 cm from its outline, so the planner, the spin and the back-up all refused. The first recovery now clears both costmaps, then backs up or, if something is really behind, turns.
- Regulated Pure Pursuit ramps from the measured speed: with `max_angular_accel` 2.0 a turn on the spot started at 0.1 rad/s (0.34 rad/s per wheel, below what moves them), the measured rate stayed 0 and so did the robot (`collision ahead` and `Failed to make progress` while turning). Now 20, the wheel ramp is in `agv_odometry`.
- A frontier still open after looking at it from 0.8 m is not unreachable: the robot now drives up to it before giving up (one run ended after 40 s because of this).

### Not done yet
- Kinect `max_height` is 0.15 m, assumed: the robot's real top height is to be measured.
- Odometry tape-measure check (postponed since 2026-11-19).

### Next steps
- 3D map with RTAB-Map (Kinect + LiDAR + odometry) during the exploration (mission mode 2), then objects and rooms.
- Localization on a saved map.

---

## 2026-11-20: Nav2 with the LiDAR

### Done
- `agv_bringup/launch/nav.launch.py` and `config/nav2.yaml`: the robot launch with SLAM plus four Nav2 servers (planner NavFn, controller Regulated Pure Pursuit, behaviors spin/back up/wait, BT navigator) and their lifecycle manager. Smoother, velocity smoother, collision monitor, waypoint follower and docking are not started: `agv_odometry` already ramps the wheel speeds.
- Robot as a circle of radius 0.17 m, inflation 0.35 m, at most 0.15 m/s and 0.6 rad/s; minimum speeds kept near the lowest tested wheel speed (2 rad/s ≈ 0.07 m/s).
- Checked first without hardware (fake TF): all plugins load and the servers become active.
- On the robot: a goal on a free floor is reached; a box on the path makes the plan go around it; a box on the left shows up on the left of the costmap, so the scan is not mirrored (closes the check postponed on 2026-11-19).

### Learned
- Regulated Pure Pursuit only follows the path (and slows or stops near obstacles); going around a new obstacle comes from the planner replanning every second. MPPI would dodge by itself but simulates ~2000 trajectories 20 times a second, too much CPU next to the Kinect and YOLO.
- The LiDAR sees only its own plane, about 13 cm above the floor. The robot drove into a lower obstacle; the wheels slipped, the odometry and then the SLAM lost the robot's position on the map, and all later goals failed until a restart.
- `odom` appears only when the wheel ESP32 has opened its micro-ROS session, a few seconds after the launch; the transform time-outs before that are harmless.

### Next steps
- Kinect depth as a second costmap layer, so low obstacles are seen.
- Save the map and navigate on it (localization instead of mapping), so a lost position can be recovered.

---

## 2026-11-16 / 2026-11-19: Whole robot in one launch, first map, Kinect in the model

### Done
- `agv_bringup`: one launch starts the micro-ROS agent, `lidar_bridge`, the robot model, `agv_odometry` and `slam_toolbox`. Driven with `teleop_twist_keyboard`, it built the map of a room.
- Checked on the robot: `i` drives forward, `j` turns left.
- LiDAR zero: a box in front of the robot showed up behind it; `lidar_bridge.yaml` now has `angle_offset_deg: -180.0`, and the map came out right after the change.
- Kinect v1 added to the model as `kinect_link`, 40 mm ahead of and 40 mm above the IMU (70 mm above `base_link`), no tilt. The driver attaches its optical frames to it, so depth and RGB are placed on the robot. Checked in RViz on 2026-11-19. The `agv_description` README was rewritten to match the Xacro (the previous one still said the IMU and LiDAR were not connected).

### Learned
- `slam_toolbox` corrects the odometry with the scans: the map closed even with the wheels slipping on a carpet.
- Two copies of the launch fight over the serial ports: the LiDAR keeps reconnecting and the micro-ROS session is recreated over and over.

### Postponed
- Tape-measure check of the odometry (1 m straight, one turn on the spot) and a direct left/right check of the scan. Not blocking while the map is right; the second will show up when Nav2 avoids obstacles.

### Next steps
- Nav2 with the LiDAR costmap: goals from RViz, obstacle avoidance.
- Then the Kinect depth as a second costmap layer (obstacles below or above the scan plane).

---

## 2026-11-12: Kinect v1 perception

### Done
- New package `kinect_v1_perception` on top of the Kinect driver (own repository, `~/kinect`): `sound_localizer` (sound detection, direction with SRP-PHAT over the 4 microphones, LED yellow on a sound), `sound_classifier` (YAMNet, 521 sound classes), `yolo_detector` (YOLO26n exported to NCNN, about 7 Hz on the Pi 5).
- Python virtual environment `~/agv_venv` for the network libraries, with NumPy kept below 2 for ROS.

### Learned
- NCNN is about 3 times faster than PyTorch on the Pi 5 for YOLO26n (106 ms against 323 ms per image).
- pip pulls setuptools ≥ 80 into the environment, which breaks `colcon build --symlink-install`; it must be removed after every install.

---

## 2026-10-21: Wheel controller on the ground (T9)

### Done
- Step test with the robot on the floor (test T9), four sets of runs. Steady state within 0.12 rad/s on both wheels; right overshoot 1-4%, left 11-18%. Accepted with the gains unchanged.
- Debug build: keys `k`/`j` for the integral limit, gains printed with every step result, filtered speed of a stopped wheel set to exactly 0 (it showed `1.4e-45` in T8).
- LiDAR frame moved in the robot model: 115 mm behind and 65 mm above the IMU.

### Learned
- On the ground the right wheel needs ~0.10 more duty than the left at 4 rad/s; the integral covers it.
- Three good runs are not proof: a lower integral limit passed once and failed after being written in `config.h`. The run-to-run spread (left 7-18%) is larger than the effect of small gain changes, and a quick calculation showed the integral never reaches the limit during the rise.
- A step from 0 to 4 rad/s in 20 ms is the worst case; in use `diff_drive_controller` limits the acceleration, and small speed differences between the wheels are measured by the odometry and corrected by the levels above (IMU, navigation), not by ever finer tuning.

### Next steps
- Wheel odometry on the Pi: `/wheel_states` → `/odom` and `/cmd_vel` → `/wheel_cmd`, then calibrate wheel radius and separation.
- Check the step response again once the Kinect is mounted.

---

## 2026-10-20: Wheel firmware, block 7 (micro-ROS) and robot model

### Done
- micro-ROS build of the wheel firmware (`pio run -e microros`): one node, `wheels_imu`, over the USB serial at 921600 baud. Out: `/wheel_states` (JointState, position and velocity) and `/imu/data_raw` (raw Imu) at 50 Hz; in: `/wheel_cmd` (JointState velocity targets). The debug build with the test keys stays as env `debug`; both share the control task, motors, encoders and IMU code.
- `micro_ros_agent` built on the Pi in `~/microros_ws` with `micro_ros_setup` (jazzy), since it is not in the Ubuntu packages.
- Test T8 passed: node visible in ROS 2, both outputs at 50.00 Hz, stamps equal to the Pi clock, targets of 4/4 and -2/+2 rad/s followed within 1%, wheels stopped by the watchdog when the commands stop, node back after an agent restart.
- First robot model, `raspberry/src/agv_description`: Xacro with chassis, wheels, `imu_link` and `laser` frames, checked in RViz2 with static wheels.

### Learned
- In the micro-ROS build a missing IMU shows up only as a topic with no messages. During T8 the BMI160 connection had come loose; it took the debug build (key `?`) to see it.
- `ros2 topic hz` and `echo` take a few seconds to start on the Pi, and their output is buffered when piped: short `timeout` runs print nothing even when the topic is fine.

### Problems
- A burst of thousands of `session re-established` lines in the agent log at one start, cause unknown; it did not come back at the next start.

### Next steps
- Publish the IMU status (found, read errors, calibration) from the micro-ROS build.
- Check the robot model with the live `/wheel_states`.
- ros2_control with `diff_drive_controller`: `/cmd_vel` in, `/odom` out.
- Retune the wheel controller on the ground; measure the encoder signal voltage.

---

## 2026-10-12: Wheel firmware, block 6 (BMI160 IMU)

### Done
- Renamed `firmware_wheels/` to `firmware_wheels_imu/`: the same ESP32 reads the IMU, and micro-ROS will publish wheels and IMU from one node.
- BMI160 driver on I2C written directly with `Wire`: detection on the configured pins (and swapped), configuration read back, ±500 °/s and ±4 g at 50 Hz, samples in m/s² and rad/s in the robot frame (REP 103), gyro bias calibration at boot.
- The IMU is read in the 20 ms control task; the longest period went to 0.53 ms, so the wheel control is not affected.
- Decision: the ESP32 publishes raw IMU data only; `robot_localization` on the Pi fuses the gyro rate with the wheel odometry. No orientation filter on either side for a ground robot.
- Test T6 passed: |a| within 2% of g, gyro noise ~0.0006 rad/s, a 90° turn read as 90.5°, board axes equal to the robot axes.

### Learned
- After a soft reset the BMI160 drops register writes that come less than 450 µs apart. With back-to-back writes the range stayed at ±2 g (acceleration read twice the real value) and the gyro read exactly 0. Fixed with a 1 ms gap after each write, and the configuration is now read back.
- The wiring diagram was hard to read on the SDA/SCL labels; the hardware confirmed SDA on GPIO21 and SCL on GPIO22.

### Next steps
- micro-ROS layer: wheel JointState, raw `sensor_msgs/Imu`, wheel speed targets.
- Retune the wheel controller on the ground (with a ~1.4 kg dummy for the Kinect v1, or after mounting it).

---

## 2026-10-11: Wheel firmware, block 5 (watchdog): core firmware done

### Done
- Command watchdog in the control task: with no speed target for 0.5 s the targets go to 0 and the motors coast. It runs in the fixed-period task, so it also covers a hung `loop()` or, later, a hung micro-ROS layer.
- The firmware accepts `v <left> <right>` lines from programs; [firmware_wheels_imu/tools/cmd_sender.py](../firmware_wheels_imu/tools/cmd_sender.py) sends them at a fixed rate. Key `w` toggles the watchdog for manual tests.
- Test T5 passed: a single command stops after 0.5 s, 20 Hz commands keep the wheels turning (~21 s), and the wheels stop within 0.5 s of the last one.
- This closes the core wheel firmware (blocks 1-5), all tested with the wheels lifted and recorded in the [test record](../firmware_wheels_imu/tests/README.md).

### Learned
- Opening the serial port restarts the ESP32 even with DTR and RTS held released by the script. Harmless (it restarts with the motors off in about a second), but the micro-ROS agent will do the same.

### Next steps
- Mount the wheels on the robot and retune the controller transients on the ground (keys `p`/`o`, `i`/`u`; start with the integral limit at 0.10).
- Refine `ENCODER_TICKS_PER_REV` by driving over a known distance.
- Add the micro-ROS layer: JointState out at ~50 Hz, wheel speed targets in through `control_set_target()`.
- Measure the encoder signal voltage with the wire disconnected (possible 5 V pull-up on a 3.3 V GPIO).

---

## 2026-10-11: Wheel firmware, block 4 (speed controller) and test record

### Done
- PI speed controller with feedforward, one per wheel, in the 20 ms control task, which is now the only code that drives the motors. `control_set_target()` in rad/s is the entry point for the future micro-ROS layer.
- Anti-windup by clamping, plus a limit on the integral; the output never opposes the target, because the encoder cannot read the direction.
- Closed-loop test keys (`7` step test, `6` now also checks the error from the target) and run-time gain tuning (`p`/`o`, `i`/`u`).
- A PC simulation of the controller ([firmware_wheels_imu/tests/sim/pid_sim.cpp](../firmware_wheels_imu/tests/sim/pid_sim.cpp)) that runs the firmware's own PI on a model of the wheel built from the tests.
- A test record ([firmware_wheels_imu/tests/README.md](../firmware_wheels_imu/tests/README.md)): every hardware test with the commit and files used, procedure, criterion, result and the output lines that show it.

### Learned
- Steady state is excellent: error ≤ 0.01 rad/s and spread ≤ 0.7% from 2 to 8 rad/s, on both wheels.
- The gains from lambda tuning (KP 0.067, KI 0.67) gave 34-35% overshoot from standstill. With a feedforward fitted on the measured duties and KI 0.20 it went down to 25-28%, still above the 10% target.
- The cause is the static friction: from standstill a wheel starts only at duty 0.35, which already means ~4.5 rad/s with no load, so below that speed it starts with a jump whatever the gains. The simulation reproduced both versions within a few percent.
- With the integral limited to ±0.20 and KI 0.20 the recovery after a hand brake got worse (+47% peak, ~1.6 s), as the simulation predicted.

### Problems
- Transients outside the agreed criteria with the wheels lifted. Decision: keep the steady-state behaviour and retune with the robot on the ground, where the inertia is much higher.

### Next steps
- Block 5: command watchdog (motors off if no command for ~0.5 s).
- Retune the transients on the ground; the simulation suggests starting with the integral limit at 0.10.

---

## 2026-10-11: Wheel firmware, block 3 (speed)

### Done
- Fixed-period FreeRTOS task on core 0 (20 ms) that computes the position and speed of each wheel.
- Speed from timed encoder edges (M/T method) plus a 40 ms low-pass filter; zero after 100 ms without edges.
- Removed the encoder counter reset: resetting the counters under the running speed estimate would produce a false speed spike. `z` now only zeroes the displayed numbers.
- Two self-evaluating test keys: `5` step test (steady speed, gain, time constant) and `6` stability test (mean, spread, PASS/FAIL).

### Learned
- Stability at duty 0.5: left 7.28 rad/s with 0.5% spread, right 6.88 rad/s with 0.8%, both equal to the tick-count mean. Counting ticks per 20 ms would have given about ±10% at this speed.
- Step test, 4 runs, very repeatable: K 14.7-15.0 (left) and 14.2-14.3 (right) rad/s per unit duty, time constant 0.10 s on both wheels.

### Next steps
- Block 4: per-wheel PI speed controller with feedforward, tuned from the step test.

---

## 2026-10-11: Wheel firmware, block 2 (motors)

### Done
- L298N driver: a duty in [-1, 1] sets direction and 1 kHz PWM; the left motor is inverted in software.
- The encoder counts are now signed: the motor driver tells the encoders which direction it drives.
- Safe reversal: on a direction change the PWM goes to 0 and the new direction is applied only after 50 ms with no encoder edge, so the ticks of the coasting wheel keep the right sign.
- Test keys on the serial monitor (1-4, +, -, z), without Enter.

### Learned
- Directions and tick signs are correct on both wheels; no glitches or fast edges with the motors on, up to full duty.
- Open-loop speed: about 7 rad/s at duty 0.5 and 11.4-11.8 rad/s at full duty. The 8 rad/s target needs a duty of about 0.6.
- From standstill both wheels start at a duty of about 0.35; once turning they keep going down to 0.10-0.15.
- The left wheel is 3-7% faster than the right one at the same duty (the earlier notes said 10-15%).

### Problems
- Line-based commands (`d 0.5 0.5` + Enter) were awkward to type in the PlatformIO monitor; replaced by single keys.

### Next steps
- Block 3: wheel speed in rad/s, plus a step test to get the motor gain and time constant for the PID.

---

## 2026-10-11: Wheel firmware, block 1 (encoders)

### Done
- Planned the wheel firmware for a second ESP32 (DevKit, 38 pins) in five blocks, each tested on the hardware before the next: encoders, motors, speed estimate, per-wheel PID, command watchdog. No ROS in this phase: everything is tested from the serial monitor. micro-ROS will be added later as a communication layer on top (JointState out, wheel speed targets in; odometry stays in `diff_drive_controller` on the Pi).
- Created `firmware_wheels/` (renamed `firmware_wheels_imu/` on 2026-10-12, as it will also read the IMU) (PlatformIO, `esp32dev`, `espressif32@6.9.0` as in the LiDAR firmware).
- Block 1: interrupt-driven counters for the two single-channel Hall encoders (CHANGE, both edges), with a 100 µs glitch filter and a "fast edge" counter (edges closer than 500 µs, impossible at the robot's speeds) to spot a noisy signal.
- Added the LiDAR and wheels wiring diagrams to the READMEs.

### Learned
- Encoder wiring: grey = GND, white = 5 V (from the ESP32 VIN), green = signal to GPIO18 (left) / GPIO19 (right).
- Measured **516.6 transitions per wheel revolution**: 4 runs of 5 revolutions turned by hand (right 511.4 and 515.6, left 524.2 and 515.0), no glitches and no fast edges. Both wheels give the same value within the hand alignment error. The iRobot Create figure (508.8) is 1.5% lower.

### Problems
- **Upload failing** with "No serial data received" right after esptool switched to 460800 baud. Uploading at 115200 works and is now set in `platformio.ini`.
- A first one-revolution test gave 840 ticks on the left wheel. Repeated over 5 revolutions it gave ~516 like the right one: it was a mistake in the procedure, not a wiring fault.

### Next steps
- Measure the encoder signal voltage with the signal wire disconnected, to rule out a 5 V pull-up on a 3.3 V GPIO.

---

## 2026-10-09: ROS 2 node for the LiDAR (`/scan`)

### Done
- **Decision: no micro-ROS for the LiDAR.** The ESP32 keeps sending the binary frames, and a ROS 2 node on the Pi turns them into `/scan`. It is less code, it reuses a protocol that is already tested, and the web viewer stays usable for debugging. micro-ROS can still be added later if the ESP32 has to do more (e.g. motors).
- Turned `raspberry/` into the ROS 2 Jazzy workspace for all the Pi nodes.
- Wrote the `lidar_bridge` package in C++: frame parser with CRC (no ROS dependency), POSIX serial port, node publishing `sensor_msgs/LaserScan` with sensor data QoS, parameter file and launch file.
- 4 unit tests (gtest) on the parser: CRC check value, garbage before a frame, frames split across reads, bad CRC followed by a good frame. All pass.
- Built with plain CMake (colcon is not installed yet) and tested with the ESP32 connected: `/scan` at 8.1 Hz, 360 rays, `scan_time` 0.122 s, ranges in metres with `inf` for no echo.
- Viewed `/scan` in RViz2 on the PC over the network: works. Needed settings: Fixed Frame `laser`, LaserScan reliability Best Effort.

### Problems
- On start-up the node reported "Lost 2739 frames": the first frame read was an old one, left in the ESP32's USB buffer while nobody was reading the port. Fixed by dropping the first frame after every (re)open.

### Next steps
- `sudo apt install python3-colcon-common-extensions` and build the workspace with colcon.
- Check the angle convention (`clockwise`, `angle_offset_deg`) with an object in front of and to the left of the LiDAR.
- Static transform `base_link → laser` once the LiDAR position on the robot is known.
- Udev rule for a stable port name (`/dev/lidar`).

---

## 2026-10-09: First LiDAR firmware, working on the hardware

### Done
- Created the PlatformIO project in `firmware_lidar/` (ESP32-C3, Arduino framework, native USB CDC) with three build environments: binary output, debug text, raw passthrough.
- Wrote the X2 packet parser (`x2_parser.*`): header sync, check bits, XOR checksum, angle interpolation across 0°, distance-based angle correction.
- Points are grouped into 360 one-degree bins per revolution (closest point wins) and sent as a binary frame with a CRC-16.
- Wrote `tools/read_scan.py` to read and validate the frames on the Pi.
- Tested the parser on the Pi (host build with g++) with synthetic packets: garbage before the header, wrap at 360°, corrupted checksum. CRC-16 in C matches Python's `binascii.crc_hqx`.
- Installed PlatformIO on the Pi (after `sudo apt install python3.12-venv`), flashed and tested the three modes on the hardware, in this order:
  - raw: about 7 KB/s, 274 `AA 55` headers in 3 s, 40 samples per packet;
  - debug: 8.0 scans/s (the LiDAR reports 7.9 Hz), about 320 of 360 bins with an echo, stable distances, 0 checksum errors;
  - binary: `read_scan.py` receives about 8 frames/s with no CRC errors and no lost frames.

- Added `tools/scan_viewer.py`, a live web viewer of the scans (the Pi is headless, the page is opened from the PC). Tested against the hardware: about 7.8 scans/s reach the page, with no CRC errors or lost frames.

### Learned
- Reading the kaiaai/LDS source: for the X2/X2L the library leaves M_SCTR **floating** (pin as input), it does not generate a PWM. The 8.3 scans/s of the first test are the LiDAR's default speed. Driving M_SCTR HIGH stops the motor. The firmware keeps the pin floating by default. Measured today with the same setup: 7.9-8.0 scans/s.
- In the ring start packet, `CT >> 1` is the scan frequency in 0.1 Hz units.

### Problems
- PlatformIO could not be installed at first: the system Python was missing the `venv` module. Fixed with `sudo apt install python3.12-venv`.
- **Upload failing** with "No serial data received" right after esptool loaded its stub. Lowering the upload speed did not help; uploading with `--no-stub` works and is now set in `platformio.ini`. Likely cause, not yet confirmed: GPIO20 is also UART0 RX, which the bootloader listens on, and the LiDAR keeps sending data into it.
- **Checksum errors when the Pi was not reading**: with the USB port closed, `Serial` writes blocked the loop for seconds and the LiDAR UART buffer overflowed (a scan rate of 0.33/s and a few checksum errors in the debug output). Fixed by writing to USB only when the whole frame fits in the buffer, and dropping it otherwise. Retested with the port closed for 8 s: 0 errors.

### Next steps
- Confirm the upload issue by flashing once with the LiDAR TX unplugged.
- micro-ROS on the ESP32 and the `/scan` topic (`sensor_msgs/LaserScan`).
- Dedicated 5V supply for the LiDAR.

---

## 2026-10-09: LiDAR identification, wiring and first test

### Done
- Salvaged the LiDAR module from a robot vacuum and identified it: label "YDS2MD-1", PCB "S-MAINBOARD-V2.1-TX_SEP", EAI logo.
- Worked out and verified the pinout of the 4-wire connector in the field (see [firmware_lidar/README.md](../firmware_lidar/README.md)).
- Wired the LiDAR to the ESP32-C3 SuperMini: TX → GPIO20, M_SCTR → GPIO3, common GND, 5V to the board's 5V pin.
- Ran a test with a throwaway sketch based on the [kaiaai/LDS](https://github.com/kaiaai/LDS) library (`LDS_YDLIDAR_X2_X2L`, 10 kHz motor PWM). Result: about 8.3 scans/s, with consistent distances and angles.
- Mechanics: designed the ESP32-C3 enclosure in OpenSCAD (USB-C slot, cable glands, 4 M3 tabs, snap-fit lid, PETG recommended) and generated the STLs. Designed the LiDAR bracket in Fusion, shaped to the actual profile of the enclosure.
- Set up the documentation structure (main README, firmware README, this log).

### Learned
- There is no public datasheet for the YDS2MD-1. The de facto reference is the YDLIDAR X2/X2L datasheet: the protocol is identical, as confirmed by real data.
- The LiDAR is single-channel: it only transmits, has no start command, and starts sending data as soon as it is powered.
- The head motor has its own 2-wire connector on the board. Power reaches the rotating head wirelessly (XKT-412 chip).
- Readings of 0.00 mm are points with no echo, not errors.

### Problems and mistakes
- **Wrong pinout at first**: the blue wire was initially taken for M_SCTR. The average voltage on an active UART TX line (about 1.7-1.8 V) looks like the default level of M_SCTR, so a multimeter is not enough. The TX line has to be identified from the data (`AA 55` header).
- **Power supply**: the ESP32 board's 5V pin is borderline for the LiDAR's inrush current (about 0.5-0.6 A). A dedicated 5V buck converter is needed.

### Next steps
- Install PlatformIO on the Raspberry Pi 5 and create the `firmware_lidar` project.
- First minimal firmware: motor PWM plus raw byte forwarding to USB, to check the whole chain.
- X2 protocol parser on the ESP32, sending each full scan as one binary frame.
- micro-ROS integration with the `/scan` topic.
- Check the LiDAR bracket: 0.2-0.3 mm diametral clearance, walls of at least 2 mm, mounting method. Measure the module with calipers, since it is an OEM part and the GrabCAD X2/G2 models are only indicative.
