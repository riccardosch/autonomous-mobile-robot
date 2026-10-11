# Development log

Newest entries go at the top. Each entry records what was done, what was learned, the problems found and the next steps.
The *current* state of each part lives in the READMEs; this file keeps the history of how it got there.

---

## 2026-10-11: Wheel firmware, block 4 (speed controller) and test record

### Done
- PI speed controller with feedforward, one per wheel, in the 20 ms control task, which is now the only code that drives the motors. `control_set_target()` in rad/s is the entry point for the future micro-ROS layer.
- Anti-windup by clamping, plus a limit on the integral; the output never opposes the target, because the encoder cannot read the direction.
- Closed-loop test keys (`7` step test, `6` now also checks the error from the target) and run-time gain tuning (`p`/`o`, `i`/`u`).
- A PC simulation of the controller ([firmware_wheels/tests/sim/pid_sim.cpp](../firmware_wheels/tests/sim/pid_sim.cpp)) that runs the firmware's own PI on a model of the wheel built from the tests.
- A test record ([firmware_wheels/tests/README.md](../firmware_wheels/tests/README.md)): every hardware test with the commit and files used, procedure, criterion, result and the output lines that show it.

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
- Created `firmware_wheels/` (PlatformIO, `esp32dev`, `espressif32@6.9.0` as in the LiDAR firmware).
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
