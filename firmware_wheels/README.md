# firmware_wheels

Firmware for the ESP32 DevKit (38 pins) that drives the two Roomba wheel motors through an L298N and reads their encoders. In this phase it has no ROS: everything is tested from the serial monitor. micro-ROS will be added later as a communication layer on top (wheel JointState out, wheel speed targets in).

> **Status (2026-10-11):** blocks 1-3 of 5 done: encoder counting, open-loop motor control and speed measurement verified on both wheels.

| Block | Contents | Status |
|---|---|---|
| 1 | Encoder counters | ✅ 516.6 transitions per revolution, no glitches |
| 2 | Motors: PWM + direction on the L298N | ✅ Directions and signed ticks correct, no glitches with the motors on |
| 3 | Wheel speed in rad/s | ✅ Spread 0.5-0.8% at ~7 rad/s; step response measured |
| 4 | Per-wheel speed PID | ⏳ |
| 5 | Command watchdog | ⏳ |

Every test run on the hardware, with the commit and files it used and the output that shows the result, is in the [test record](tests/README.md).

## Wiring

See the diagram and pin table in the [main README](../README.md#wheels). All pins are set in [src/config.h](src/config.h).

## Encoders

Each Roomba wheel module has a **single-channel** Hall encoder (not quadrature), so the direction cannot be read from the signal: it is taken from the commanded motor direction (see [Motors](#motors)).

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

## Motors

The L298N is driven with a 1 kHz, 10-bit PWM on ENA/ENB and the direction on IN1-IN4. A duty in [-1, 1] sets both: the sign is the direction (+ = robot forward), the magnitude the PWM.

- `MOTOR_LEFT_INVERTED` swaps IN1/IN2 in software: the left motor is mounted mirrored.
- Duty 0 lets the wheel coast (`MOTOR_BRAKE_ON_STOP = true` brakes instead).
- **Safe reversal.** The encoder cannot read the direction, so the motor driver tells it which sign to count. When the direction changes while the wheel is still turning, the PWM goes to 0 at once, but the new direction is applied only after 50 ms with no encoder edge. The ticks of the coasting wheel are counted with the old sign.

Open-loop results (2026-10-11, wheels lifted):

| Duty | Left | Right |
|---|---|---|
| 0.40 | 5.5 rad/s | 5.1 rad/s |
| 0.50 | 7.3 rad/s | 6.9 rad/s |
| 0.60 | 8.5 rad/s | 8.1 rad/s |
| 0.70 | 9.4 rad/s | 8.9 rad/s |
| 1.00 | 11.8 rad/s | 11.4 rad/s |

- From standstill both wheels start at a duty of about **0.35**; once turning they keep going down to about 0.10-0.15 (static friction is higher than sliding friction).
- The left wheel is 3-7% faster than the right one at the same duty.
- The 8 rad/s target speed needs a duty of about 0.6, which leaves margin for the speed controller.

## Speed estimate

A FreeRTOS task on core 0 runs every 20 ms (`vTaskDelayUntil`), reads the encoders and computes the position (rad) and speed (rad/s) of each wheel. `loop()` and, later, micro-ROS read them with `control_get_state()`.

- **Timed edges (M/T method).** The speed is the ticks between the last edge of the previous period and the last edge of this one, divided by the time between those two edges. At 3 rad/s only ~5 ticks fall in 20 ms, so counting ticks per period would give ±20%; timing the edges removes that error.
- A first-order low-pass filter with a 40 ms time constant smooths what is left.
- With no edge for 100 ms the speed is 0 (slowest measurable speed ~0.12 rad/s). Between edges the estimate can only decrease, so a wheel that stops is seen as slowing down.
- The encoder counters are never reset; `z` only zeroes the displayed numbers.

Results (2026-10-11, wheels lifted, duty 0.5):

| | Stability test (5 s) | Step test (4 runs) |
|---|---|---|
| Left | 7.28 rad/s, std 0.5%, equal to the tick-count mean | steady 7.37-7.49 rad/s, K 14.7-15.0 rad/s per duty, τ 0.10 s |
| Right | 6.88 rad/s, std 0.8%, equal to the tick-count mean | steady 7.09-7.14 rad/s, K 14.2-14.3 rad/s per duty, τ 0.10 s |

K is the steady speed divided by the duty; τ is the time to reach 63% of the steady speed (20 ms resolution).

## Build and flash

The board shows up on the Pi as `/dev/ttyUSB0`. Pass the port explicitly, so that PlatformIO does not pick the LiDAR's ESP32-C3 (`/dev/ttyACM0`).

```bash
cd firmware_wheels
~/.platformio/penv/bin/pio run -t upload --upload-port /dev/ttyUSB0
~/.platformio/penv/bin/pio device monitor --port /dev/ttyUSB0   # Ctrl+C to exit
```

**Upload speed.** At the default 460800 baud the upload fails right after the stub starts ("No serial data received"). `platformio.ini` sets `upload_speed = 115200`, which works.

## Serial output and test keys

Every 200 ms, one line per wheel pair (`w` is the filtered speed):

```
L   1017 ticks  w  +7.21 rad/s  duty +0.50 (req +0.50)  glitch 0 fast 0 | R    964 ticks  w  +6.83 rad/s  duty +0.50 (req +0.50)  glitch 0 fast 0
```

`duty` is the value applied to the bridge, `req` the requested one: they differ while a reversal waits for the wheel to stop.

Single keys, no Enter needed:

| Key | Action |
|---|---|
| `1` | Both wheels forward at duty 0.5 |
| `2` | Both wheels backward at duty 0.5 |
| `3` | Left forward, right backward |
| `4` | Stop |
| `+` / `-` | Both wheels forward, duty up / down by 0.05 |
| `5` | Step test: duty 0.5 from standstill for 2 s, prints steady speed, gain K and time constant τ |
| `6` | Stability test: with the wheels turning, measures for 5 s and prints mean, spread and PASS/FAIL |
| `z` | Zero the displayed counters |

While a test runs the periodic output stops and only `4` (abort) is accepted.

**Encoder test:** motors off, mark the wheel, press `z`, turn the wheel 5 revolutions by hand and stop on the mark. Expected: about 2580 ticks, `glitch 0`, `fast 0`.

**Motor test** (wheels lifted): `1` turns both wheels forward with rising ticks, `2` backward with falling ticks, `3` in opposite directions, `4` stops them. `glitch` and `fast` stay at 0.

**Speed test** (wheels lifted): `1`, wait 2 s, `6`. Pass: std ≤ 3% of the mean and mean within 1% of the tick-count mean, on both wheels.

## Code layout

| File | Contents |
|---|---|
| [platformio.ini](platformio.ini) | Board, platform version, upload and monitor speed |
| [src/config.h](src/config.h) | Pins and constants |
| [src/encoders.h](src/encoders.h), [src/encoders.cpp](src/encoders.cpp) | Encoder interrupts, consistent snapshot of counts and edge times |
| [src/motors.h](src/motors.h), [src/motors.cpp](src/motors.cpp) | L298N driver: duty to PWM and direction, safe reversal |
| [src/velocity.h](src/velocity.h), [src/velocity.cpp](src/velocity.cpp) | Speed estimate: timed edges + low-pass filter |
| [src/control.h](src/control.h), [src/control.cpp](src/control.cpp) | Fixed-period task: position and speed of each wheel, recording for the tests |
| [src/main.cpp](src/main.cpp) | Setup, test keys and their analysis, debug output |
