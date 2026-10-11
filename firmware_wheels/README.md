# firmware_wheels

Firmware for the ESP32 DevKit (38 pins) that drives the two Roomba wheel motors through an L298N and reads their encoders. In this phase it has no ROS: everything is tested from the serial monitor. micro-ROS will be added later as a communication layer on top (wheel JointState out, wheel speed targets in).

> **Status (2026-10-11):** block 1 of 5 done, encoder counting verified on both wheels.

| Block | Contents | Status |
|---|---|---|
| 1 | Encoder counters | ✅ 516.6 transitions per revolution, no glitches |
| 2 | Motors: PWM + direction on the L298N | ⏳ |
| 3 | Wheel speed in rad/s | ⏳ |
| 4 | Per-wheel speed PID | ⏳ |
| 5 | Command watchdog | ⏳ |

## Wiring

See the diagram and pin table in the [main README](../README.md#wheels). All pins are set in [src/config.h](src/config.h).

## Encoders

Each Roomba wheel module has a **single-channel** Hall encoder (not quadrature), so the direction cannot be read from the signal: it will be taken from the commanded motor direction (block 2).

| Wire | Signal |
|---|---|
| Grey | GND |
| White | +5 V (ESP32 VIN) |
| Green | Signal, to GPIO18 (left) / GPIO19 (right), `INPUT_PULLUP` |

- An interrupt on every edge (CHANGE) counts one transition and stores its time.
- Edges less than 100 µs after the previous one are dropped as noise (`glitch` counter).
- Accepted edges less than 500 µs apart are counted as `fast`. At the maximum wheel speed (8 rad/s) edges are ~1.5 ms apart, so a fast edge means a noisy signal.

**Transitions per revolution: 516.6**, measured by hand on 2026-10-11. Each run is 5 revolutions, stopping on a mark:

| Wheel | Slow | Normal speed |
|---|---|---|
| Right | 2557 (511.4 /rev) | 2578 (515.6 /rev) |
| Left | 2621 (524.2 /rev) | 2575 (515.0 /rev) |

No glitches and no fast edges. The spread comes from the hand alignment; the two wheels give the same value. The iRobot Create figure (508.8) is 1.5% lower. The value can be refined later by driving over a known distance.

## Build and flash

The board shows up on the Pi as `/dev/ttyUSB0`. Pass the port explicitly, so that PlatformIO does not pick the LiDAR's ESP32-C3 (`/dev/ttyACM0`).

```bash
cd firmware_wheels
~/.platformio/penv/bin/pio run -t upload --upload-port /dev/ttyUSB0
~/.platformio/penv/bin/pio device monitor --port /dev/ttyUSB0   # Ctrl+C to exit
```

**Upload speed.** At the default 460800 baud the upload fails right after the stub starts ("No serial data received"). `platformio.ini` sets `upload_speed = 115200`, which works.

## Serial output (block 1)

Every 200 ms:

```
L   2575 ticks   5.061 rev glitch 0 fast 0 | R      0 ticks   0.000 rev glitch 0 fast 0
```

Send `z` to reset the counters.

**Encoder test:** mark the wheel, press `z`, turn the wheel 5 revolutions by hand and stop on the mark. Expected: about 2580 ticks, `glitch 0`, `fast 0`.

## Code layout

| File | Contents |
|---|---|
| [platformio.ini](platformio.ini) | Board, platform version, upload and monitor speed |
| [src/config.h](src/config.h) | Pins and constants |
| [src/encoders.h](src/encoders.h), [src/encoders.cpp](src/encoders.cpp) | Encoder interrupts, consistent snapshot of counts and edge times |
| [src/main.cpp](src/main.cpp) | Setup, serial commands, debug output |
