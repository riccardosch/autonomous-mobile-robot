# firmware_wheels_imu

Firmware for the ESP32 DevKit (38 pins) that drives the two Roomba wheel motors through an L298N and reads their encoders. In this phase it has no ROS: everything is tested from the serial monitor. micro-ROS will be added later as a communication layer on top (wheel JointState out, wheel speed targets in).

> **Status (2026-10-11):** all 5 blocks of the core firmware done and tested with the wheels lifted. The speed controller holds the target within 0.01 rad/s from 2 to 8 rad/s and the watchdog stops the wheels 0.5 s after the last command. Still open: the controller transients (overshoot from standstill, recovery after a hand brake), to retune with the robot on the ground; then the micro-ROS layer.

| Block | Contents | Status |
|---|---|---|
| 1 | Encoder counters | ✅ 516.6 transitions per revolution, no glitches |
| 2 | Motors: PWM + direction on the L298N | ✅ Directions and signed ticks correct, no glitches with the motors on |
| 3 | Wheel speed in rad/s | ✅ Spread 0.5-0.8% at ~7 rad/s; step response measured |
| 4 | Per-wheel speed PID | ✅ Steady state within 0.01 rad/s, spread ≤ 0.4%. ⚠️ Overshoot 25-28% from standstill (target 10%): retune on the ground |
| 5 | Command watchdog | ✅ Wheels stop 0.5 s after the last target |

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

## Speed controller

The control task runs one PI with feedforward per wheel, every 20 ms:

duty = feedforward + KP · error + integral, with feedforward = FF_OFFSET + |target| / FF_GAIN (with the sign of the target).

| Parameter | Value | Where it comes from |
|---|---|---|
| `PID_KP` | 0.067 duty per rad/s | Lambda tuning on the step test (K ≈ 15 rad/s per duty, τ = 0.10 s): KP = 1 / K |
| `PID_KI` | 0.20 duty per rad | Lowered from KP / τ = 0.67, which gave 34% overshoot |
| `PID_I_MAX` | ±0.20 duty | Limits the integral during a brake; also leaves room for a ~20% lower battery voltage |
| `FF_OFFSET_LEFT/RIGHT` | 0.14 / 0.125 duty | Straight-line fit of the steady closed-loop duty at 2, 4, 6 and 8 rad/s |
| `FF_GAIN_LEFT/RIGHT` | 21.6 / 19.0 rad/s per duty | Same fit; within 0.02 duty of every measured point |

- **Anti-windup:** the integral stops when the duty is saturated and while a motor waits for its wheel to stop before reversing, and is limited to ±`PID_I_MAX`.
- **Never against the target:** the duty is kept on the side of the target (≥ 0 for a forward target). The encoder cannot read the direction, so the controller never brakes by reversing: a wheel that is too fast coasts down.
- A target of 0 switches the motor off and clears the integral.
- `control_set_target(left, right)` in rad/s is the entry point the micro-ROS layer will use; `control_set_duty()` stays for the open-loop tests.

**Known limits (wheels lifted, 2026-10-11):**

- From standstill a wheel starts only at duty 0.35, which already gives ~4.5 rad/s with no load. For targets below that it starts with a jump: 25-28% overshoot on a 0 → 4 rad/s step whatever the gains (see the simulation in [tests/sim/](tests/sim/pid_sim.cpp)).
- After a hand brake the integral stays at its limit and, with KI = 0.20, empties slowly: +47% peak and ~1.6 s back to the target.
- Without the robot's weight the inertia is far lower than on the floor, so the transients will be retuned with the robot on the ground. Keys `p`/`o` and `i`/`u` change KP and KI at run time for that.

## Command watchdog

In closed loop every `control_set_target()` call restarts a 0.5 s timer (`CMD_TIMEOUT_S`). When it runs out the control task itself sets both targets to 0 and the motors coast, so the wheels stop even if `loop()` or, later, the micro-ROS layer hangs. Open-loop duties (test key `5` only) are not watched.

- On at boot (`CMD_WATCHDOG_ON_AT_BOOT`); key `w` toggles it for manual tests. With it on, keys `1`-`3` and `+`/`-` drive the wheels for 0.5 s only, and tests `6` and `7` refuse to start.
- When it trips the firmware prints `watchdog: no target for 0.50 s, motors off`.

### Commands from a script

The firmware also accepts lines `v <left> <right>` (rad/s, ended by a newline). They are meant for programs, not for typing. [tools/cmd_sender.py](tools/cmd_sender.py) sends them at a fixed rate and prints what the ESP32 writes back; after Ctrl+C it keeps printing for 3 s, so you can see the watchdog stop the wheels:

```bash
python3 tools/cmd_sender.py                       # 4 rad/s on both wheels, 20 Hz
python3 tools/cmd_sender.py --left 2 --right -2 --rate 10
```

Close `pio device monitor` first: only one program at a time can open the port.

**Opening the port resets the ESP32**, even with DTR and RTS held released by the script: the firmware restarts in about a second with the motors off. The same will happen when the micro-ROS agent opens the port.

## Build and flash

The board shows up on the Pi as `/dev/ttyUSB0`. Pass the port explicitly, so that PlatformIO does not pick the LiDAR's ESP32-C3 (`/dev/ttyACM0`).

```bash
cd firmware_wheels_imu
~/.platformio/penv/bin/pio run -t upload --upload-port /dev/ttyUSB0
~/.platformio/penv/bin/pio device monitor --port /dev/ttyUSB0   # Ctrl+C to exit
```

**Upload speed.** At the default 460800 baud the upload fails right after the stub starts ("No serial data received"). `platformio.ini` sets `upload_speed = 115200`, which works.

## Serial output and test keys

Every 200 ms, one line per wheel pair (`w` is the filtered speed, `duty` the value applied to the bridge):

```
L   4846 ticks  w  +4.00 rad/s  target +4.00  duty +0.31  glitch 0 fast 0 | R   4845 ticks  w  +4.00 rad/s  target +4.00  duty +0.33  glitch 0 fast 0
```

Single keys, no Enter needed:

| Key | Action |
|---|---|
| `1` | Both wheels forward at 4 rad/s |
| `2` | Both wheels backward at 4 rad/s |
| `3` | Left forward, right backward, 4 rad/s |
| `4` | Stop (also aborts a test) |
| `+` / `-` | Both wheels forward, target up / down by 1 rad/s (0 to 8) |
| `5` | Open-loop step test: duty 0.5 from standstill for 2 s, prints steady speed, gain K and time constant τ |
| `6` | Stability test: with the wheels turning, measures for 5 s and prints mean, spread, error from the target and PASS/FAIL |
| `7` | Closed-loop step test: 4 rad/s from standstill for 2 s, prints 10-90% rise time, overshoot, steady error and PASS/FAIL |
| `p` / `o` | KP × / ÷ 1.2 |
| `i` / `u` | KI × / ÷ 1.2 |
| `w` | Watchdog on / off (on at boot) |
| `z` | Zero the displayed counters |

Gains changed with `p`/`o`/`i`/`u` go back to the `config.h` values at reset.

While a test runs the periodic output stops and only `4` (abort) is accepted.

**Encoder test:** motors off, mark the wheel, press `z`, turn the wheel 5 revolutions by hand and stop on the mark. Expected: about 2580 ticks, `glitch 0`, `fast 0`.

**Motor test** (wheels lifted): `1` turns both wheels forward with rising ticks, `2` backward with falling ticks, `3` in opposite directions, `4` stops them. `glitch` and `fast` stay at 0.

**Speed test** (wheels lifted): `1`, wait 2 s, `6`. Pass: std ≤ 3% of the mean, mean within 1% of the tick-count mean and within max(3%, 0.1 rad/s) of the target, on both wheels.

**Controller step test** (wheels lifted): `w` (watchdog off), then `7` from standstill. Pass: rise ≤ 0.3 s, overshoot ≤ 10%, steady error within max(3%, 0.1 rad/s).

**Watchdog test** (wheels lifted): with the watchdog on, `1` must stop by itself after ~0.5 s; `tools/cmd_sender.py` must keep the wheels turning, and Ctrl+C on it must stop them within 0.5 s.

Blocks 2 and 3 were tested with earlier builds where keys 1-3 and `+`/`-` set the duty directly; see the [test record](tests/README.md).

## Code layout

| File | Contents |
|---|---|
| [platformio.ini](platformio.ini) | Board, platform version, upload and monitor speed |
| [src/config.h](src/config.h) | Pins and constants |
| [src/encoders.h](src/encoders.h), [src/encoders.cpp](src/encoders.cpp) | Encoder interrupts, consistent snapshot of counts and edge times |
| [src/motors.h](src/motors.h), [src/motors.cpp](src/motors.cpp) | L298N driver: duty to PWM and direction, safe reversal |
| [src/velocity.h](src/velocity.h), [src/velocity.cpp](src/velocity.cpp) | Speed estimate: timed edges + low-pass filter |
| [src/pid.h](src/pid.h), [src/pid.cpp](src/pid.cpp) | PI with feedforward, integral limit and clamping anti-windup; plain C++, also built on the PC by the simulation |
| [src/control.h](src/control.h), [src/control.cpp](src/control.cpp) | Fixed-period task: speed estimate, speed controller, motor commands, recording for the tests |
| [tools/cmd_sender.py](tools/cmd_sender.py) | Sends `v <left> <right>` targets at a fixed rate, for the watchdog test |
| [tests/README.md](tests/README.md) | Test record |
| [tests/sim/pid_sim.cpp](tests/sim/pid_sim.cpp) | PC simulation of the controller on a model of the wheel built from the tests |
| [src/main.cpp](src/main.cpp) | Setup, test keys and their analysis, debug output |
