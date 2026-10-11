# Wheel firmware test record

Every test run on the wheel firmware, with the code it was run on, the procedure, the pass criterion and the serial output lines that show the result. All tests so far were run with the wheels lifted off the ground, the ESP32 on `/dev/ttyUSB0` and the PlatformIO serial monitor (see [../README.md](../README.md#build-and-flash)).

| ID | Date | Block | What it checks | Result |
|---|---|---|---|---|
| [T1](#t1-encoder-counts-by-hand) | 2026-10-11 | 1 | Encoder transitions per wheel revolution | ✅ 516.6 /rev |
| [T2](#t2-motor-directions-and-dead-zone) | 2026-10-11 | 2 | Motor directions, tick signs, reversal, dead zone | ✅ |
| [T3](#t3-speed-estimate) | 2026-10-11 | 3 | Speed estimate stability, step response | ✅ PASS |
| [T4](#t4-speed-controller) | 2026-10-11 | 4 | Speed controller: steady state, step, hand brake | ✅ steady state / ❌ transients (retune on the ground) |
| [T5](#t5-command-watchdog) | 2026-10-11 | 5 | Wheels stop when the commands stop | ✅ PASS |

---

## T1: Encoder counts by hand

| | |
|---|---|
| Firmware | commit `16be507` (block 1). Files: [src/encoders.cpp](../src/encoders.cpp), [src/main.cpp](../src/main.cpp), [src/config.h](../src/config.h) |
| Procedure | Motors off. Mark on the wheel aligned with a fixed mark, `z`, turn the wheel 5 revolutions by hand, stop on the mark. Twice per wheel: slowly (~2 s per revolution) and at ~1 revolution per second |
| Criterion | About the same number of ticks per revolution on both wheels and at both speeds; `glitch 0` and `fast 0` |
| Result | ✅ Mean 516.6 transitions per revolution, used as `ENCODER_TICKS_PER_REV` |

| Run | Ticks | Per revolution |
|---|---|---|
| Right, slow | 2557 | 511.4 |
| Right, normal | 2578 | 515.6 |
| Left, slow | 2621 | 524.2 |
| Left, normal | 2575 | 515.0 |

Evidence (one line per run; `rev` was computed with the placeholder 508.8):

```
L      0 ticks   0.000 rev glitch 0 fast 0 | R   2557 ticks   5.026 rev glitch 0 fast 0
L      0 ticks   0.000 rev glitch 0 fast 0 | R   2578 ticks   5.067 rev glitch 0 fast 0
L   2621 ticks   5.151 rev glitch 0 fast 0 | R      0 ticks   0.000 rev glitch 0 fast 0
L   2575 ticks   5.061 rev glitch 0 fast 0 | R      0 ticks   0.000 rev glitch 0 fast 0
```

Notes: a first one-revolution run gave 840 ticks on the left wheel; the 5-revolution runs above showed it was a mistake in the procedure. That first run used a build without the `fast` counter.

---

## T2: Motor directions and dead zone

| | |
|---|---|
| Firmware | commit `616d4a5` (block 2). Files: [src/motors.cpp](../src/motors.cpp), [src/encoders.cpp](../src/encoders.cpp) (signed counts), [src/main.cpp](../src/main.cpp) (keys), [src/config.h](../src/config.h) |
| Procedure | `z`, then `1` (forward), `2` (backward), `3` (opposite), `4` (stop); then `4` and `+` one step at a time from duty 0 up to 1 |
| Criterion | Key `1` turns both wheels forward (checked by eye) with rising ticks, `2` backward with falling ticks, `3` left forward and right backward; a reversal waits for the wheel to stop; `glitch 0` and `fast 0` with the motors on |
| Result | ✅ All directions and signs correct, no glitches up to full duty. Start from standstill at duty ~0.35 |

Forward at duty 0.5, ticks rising on both wheels:

```
L    938 ticks   1.816 rev duty +0.50 (req +0.50) glitch 0 fast 0 | R    884 ticks   1.711 rev duty +0.50 (req +0.50) glitch 0 fast 0
L   1059 ticks   2.050 rev duty +0.50 (req +0.50) glitch 0 fast 0 | R    996 ticks   1.928 rev duty +0.50 (req +0.50) glitch 0 fast 0
```

Reversal (`2` while turning forward): the duty stays at 0 until the wheel stops, the coasting ticks keep the forward sign, then the ticks fall:

```
L   2385 ticks   4.617 rev duty +0.00 (req -0.50) glitch 0 fast 0 | R   2055 ticks   3.978 rev duty +0.00 (req -0.50) glitch 0 fast 0
L   2355 ticks   4.559 rev duty -0.50 (req -0.50) glitch 0 fast 0 | R   2017 ticks   3.904 rev duty -0.50 (req -0.50) glitch 0 fast 0
```

Opposite directions (`3`):

```
L   1035 ticks   2.003 rev duty +0.50 (req +0.50) glitch 0 fast 0 | R  -3478 ticks  -6.732 rev duty -0.50 (req -0.50) glitch 0 fast 0
```

Dead zone (`+`): no motion at 0.30, both wheels start at 0.35:

```
L      0 ticks   0.000 rev duty +0.30 (req +0.30) glitch 0 fast 0 | R      0 ticks   0.000 rev duty +0.30 (req +0.30) glitch 0 fast 0
L     32 ticks   0.062 rev duty +0.35 (req +0.35) glitch 0 fast 0 | R      9 ticks   0.017 rev duty +0.35 (req +0.35) glitch 0 fast 0
```

Full duty, 193 and 188 ticks per 200 ms (11.8 and 11.4 rad/s):

```
L   3889 ticks   7.528 rev duty +1.00 (req +1.00) glitch 0 fast 0 | R   3706 ticks   7.174 rev duty +1.00 (req +1.00) glitch 0 fast 0
L   4082 ticks   7.902 rev duty +1.00 (req +1.00) glitch 0 fast 0 | R   3894 ticks   7.538 rev duty +1.00 (req +1.00) glitch 0 fast 0
```

Speed against duty, from the tick increments every 200 ms:

| Duty | Left | Right |
|---|---|---|
| 0.40 | 5.5 rad/s | 5.1 rad/s |
| 0.50 | 7.3 rad/s | 6.9 rad/s |
| 0.60 | 8.5 rad/s | 8.1 rad/s |
| 0.70 | 9.4 rad/s | 8.9 rad/s |
| 1.00 | 11.8 rad/s | 11.4 rad/s |

Going down with `-`, the wheels keep turning to about 0.10-0.15.

---

## T3: Speed estimate

| | |
|---|---|
| Firmware | commit `a6668f5` (block 3). Files: [src/velocity.cpp](../src/velocity.cpp), [src/control.cpp](../src/control.cpp), [src/main.cpp](../src/main.cpp) (keys 5 and 6 and their analysis), [src/config.h](../src/config.h) |
| Procedure | **T3.1 stability:** `1` (duty 0.5), wait 2 s, `6`. **T3.2 step response:** `4`, wait for standstill, `5`; repeated 4 times |
| Criterion | T3.1: on each wheel std ≤ 3% of the mean and mean within 1% of the mean from the tick counts. T3.2: repeatable steady speed, gain and time constant |
| Result | ✅ T3.1 PASS (std 0.5% and 0.8%). T3.2: K 14.7-15.0 / 14.2-14.3 rad/s per duty, τ 0.10 s, used to tune the controller |

T3.1:

```
stability test result (5.0 s, 250 samples):
  L  mean  +7.28 rad/s  std 0.035 (0.5%)  from ticks  +7.28 (diff 0.0%)  PASS
  R  mean  +6.88 rad/s  std 0.052 (0.8%)  from ticks  +6.88 (diff 0.0%)  PASS
```

T3.2, four runs:

```
  L  steady  +7.49 rad/s  gain K  15.0 rad/s per duty  tau 0.10 s
  R  steady  +7.09 rad/s  gain K  14.2 rad/s per duty  tau 0.10 s
  L  steady  +7.39 rad/s  gain K  14.8 rad/s per duty  tau 0.10 s
  R  steady  +7.10 rad/s  gain K  14.2 rad/s per duty  tau 0.10 s
  L  steady  +7.37 rad/s  gain K  14.7 rad/s per duty  tau 0.10 s
  R  steady  +7.12 rad/s  gain K  14.2 rad/s per duty  tau 0.10 s
  L  steady  +7.40 rad/s  gain K  14.8 rad/s per duty  tau 0.10 s
  R  steady  +7.14 rad/s  gain K  14.3 rad/s per duty  tau 0.10 s
```

---

## T4: Speed controller

Four steps: the first version on the hardware, a PC simulation to understand its overshoot, a measurement for the feedforward, and the final version on the hardware.

**Criteria** (agreed before the block): steady error within max(3%, 0.1 rad/s) of the target at 2-8 rad/s; 0 → 4 rad/s step with 10-90% rise ≤ 0.3 s and overshoot ≤ 10%; after a hand brake, back to the target within ~0.5 s without a speed peak.

| | Steady state 2-8 rad/s | Step 0 → 4 rad/s | Hand brake at 4 rad/s |
|---|---|---|---|
| T4.1 first version | ✅ error ≤ 0.01 rad/s | ❌ overshoot 34-35% | ❌ peak +26%, back in ~0.4 s |
| T4.4 final version | ✅ error ≤ 0.01 rad/s | ❌ overshoot 25-28% | ❌ peak +47%, back in ~1.6 s |

The transients fail with the wheels lifted. Two reasons, both reproduced by the simulation (T4.2):

- **Static friction.** From standstill a wheel starts only at duty 0.35, which already gives ~4.5 rad/s with no load. Below that speed it starts with a jump, whatever the gains.
- **No load.** Without the robot's weight the inertia is far lower than on the floor.

Decision (2026-10-11): keep the steady-state behaviour, which is what odometry needs, move on to block 5, and retune the transients with the robot on the ground (keys `p`/`o`, `i`/`u`). Starting point suggested by the simulation: `PID_I_MAX` 0.10 to shorten the recovery after a brake.

### T4.1: First version on the hardware

| | |
|---|---|
| Firmware | Build of block 4 before commit `ae436fe`, with KP 0.067, KI 0.67 (lambda tuning: KP = 1/K, KI = KP/τ), no integral limit, feedforward target / 14.8 (left) and / 14.2 (right). Files: [src/pid.cpp](../src/pid.cpp), [src/control.cpp](../src/control.cpp), [src/main.cpp](../src/main.cpp) (keys 6 and 7), [src/config.h](../src/config.h) |
| Procedure | `7` from standstill; `+` to 2, 4, 6 and 8 rad/s and `6` at each; at 4 rad/s, left wheel braked by hand for ~2 s and released |
| Result | Steady state ✅ at every speed. Step ❌ 34-35% overshoot. Brake ❌ duty up to 0.87, peak 5.06 rad/s after the release |

```
closed-loop step result (target 4.0 rad/s, time resolution 0.02 s):
  L  rise 0.08 s  overshoot 33.8%  steady  +4.00 rad/s (error +0.00)  FAIL
  R  rise 0.08 s  overshoot 35.3%  steady  +4.00 rad/s (error -0.00)  FAIL
```

Stability at 2, 4, 6 and 8 rad/s:

```
  L  mean  +2.00 rad/s  std 0.013 (0.7%)  from ticks  +2.00 (diff 0.0%)  target  +2.00 (error -0.00)  PASS
  R  mean  +2.00 rad/s  std 0.008 (0.4%)  from ticks  +2.00 (diff 0.0%)  target  +2.00 (error +0.00)  PASS
  L  mean  +4.00 rad/s  std 0.017 (0.4%)  from ticks  +4.00 (diff 0.0%)  target  +4.00 (error +0.00)  PASS
  R  mean  +4.00 rad/s  std 0.008 (0.2%)  from ticks  +4.00 (diff 0.0%)  target  +4.00 (error +0.00)  PASS
  L  mean  +6.00 rad/s  std 0.010 (0.2%)  from ticks  +6.00 (diff 0.0%)  target  +6.00 (error +0.00)  PASS
  R  mean  +6.00 rad/s  std 0.007 (0.1%)  from ticks  +6.00 (diff 0.0%)  target  +6.00 (error +0.00)  PASS
  L  mean  +8.00 rad/s  std 0.013 (0.2%)  from ticks  +8.00 (diff 0.0%)  target  +8.00 (error +0.00)  PASS
  R  mean  +8.00 rad/s  std 0.012 (0.2%)  from ticks  +8.00 (diff 0.0%)  target  +8.00 (error +0.00)  PASS
```

Hand brake on the left wheel (one line every 200 ms): braked, integral winding up, released, peak:

```
L  33836 ticks  w  +2.62 rad/s  target +4.00  duty +0.50  glitch 0 fast 0 | R  33854 ticks  w  +4.00 rad/s  target +4.00  duty +0.33  glitch 0 fast 0
L  33987 ticks  w  +2.92 rad/s  target +4.00  duty +0.87  glitch 0 fast 0 | R  34051 ticks  w  +3.95 rad/s  target +4.00  duty +0.34  glitch 0 fast 0
L  34334 ticks  w  +5.06 rad/s  target +4.00  duty +0.65  glitch 0 fast 0 | R  34380 ticks  w  +3.99 rad/s  target +4.00  duty +0.33  glitch 0 fast 0
L  34407 ticks  w  +4.08 rad/s  target +4.00  duty +0.61  glitch 0 fast 0 | R  34446 ticks  w  +4.02 rad/s  target +4.00  duty +0.33  glitch 0 fast 0
```

### T4.2: PC simulation

| | |
|---|---|
| Code | [sim/pid_sim.cpp](sim/pid_sim.cpp) (commit `ae436fe`): the firmware's own [src/pid.cpp](../src/pid.cpp) with the values of [src/config.h](../src/config.h), on a model of the left wheel built from T2, T3 and T4.3 (speed-duty curve, τ 0.10 s, start at duty 0.35, same speed filter and period) |
| Run | From `firmware_wheels/`: `g++ -O2 -Isrc tests/sim/pid_sim.cpp src/pid.cpp -o /tmp/pid_sim && /tmp/pid_sim` |
| Result | It reproduces the hardware: 31% overshoot predicted for T4.1 (34-35% measured); +47% peak and ~1.5 s recovery predicted for T4.4 (+47% and ~1.6 s measured). It showed that no choice of KP and KI meets every criterion with the wheels lifted, because of the static friction |

First version (`/tmp/pid_sim 0.067 0.67 1 0 14.8`):

```
  0 -> 4 rad/s: rise 0.08 s  overshoot  30.9%  error -0.00 | after brake: peak +92.7%  settled in 0.58 s
```

Final version (`/tmp/pid_sim`):

```
KP 0.067  KI 0.200  integral limit 0.20  feedforward 0.140 + target / 21.6
  0 -> 2 rad/s: rise 0.08 s  overshoot  15.6%  error +0.00 | after brake: peak +17.4%  settled in 1.70 s
  0 -> 4 rad/s: rise 0.08 s  overshoot  19.5%  error +0.01 | after brake: peak +47.2%  settled in 1.52 s
  0 -> 8 rad/s: rise 0.14 s  overshoot   5.6%  error +0.06 | after brake: peak +12.0%  settled in 1.52 s
  2 -> 6 rad/s: rise 0.10 s  overshoot  12.7%  error +0.03 | after brake: peak +27.4%  settled in 1.48 s
  4 -> 8 rad/s: rise 0.12 s  overshoot   3.7%  error +0.04 | after brake: peak +12.0%  settled in 1.52 s
```

The simulated hand brake is harder than the real one, so the brake peak of the first version is overstated (93% against 26% measured).

### T4.3: Duty needed at each speed (feedforward)

| | |
|---|---|
| Firmware | Same as T4.1 |
| Procedure | `+` up to 2, 4, 6 and 8 rad/s, reading the steady `duty` column |
| Result | Fit duty = offset + target / gain: left 0.14 + target / 21.6, right 0.125 + target / 19.0, within 0.02 duty of every point. Used as `FF_OFFSET_*` and `FF_GAIN_*` |

| Target | Left | Right |
|---|---|---|
| 2 rad/s | 0.24 | 0.24 |
| 4 rad/s | 0.31-0.32 | 0.33 |
| 6 rad/s | 0.40 | 0.42-0.43 |
| 8 rad/s | 0.52 | 0.56 |

```
L  34866 ticks  w  +2.01 rad/s  target +2.00  duty +0.24  glitch 0 fast 0 | R  34915 ticks  w  +2.01 rad/s  target +2.00  duty +0.24  glitch 0 fast 0
L  35501 ticks  w  +4.00 rad/s  target +4.00  duty +0.31  glitch 0 fast 0 | R  35551 ticks  w  +4.00 rad/s  target +4.00  duty +0.33  glitch 0 fast 0
L  36955 ticks  w  +6.00 rad/s  target +6.00  duty +0.40  glitch 0 fast 0 | R  37003 ticks  w  +5.99 rad/s  target +6.00  duty +0.42  glitch 0 fast 0
L  39915 ticks  w  +8.00 rad/s  target +8.00  duty +0.52  glitch 0 fast 0 | R  39962 ticks  w  +8.00 rad/s  target +8.00  duty +0.56  glitch 0 fast 0
```

### T4.4: Final version on the hardware

| | |
|---|---|
| Firmware | commit `ae436fe`: KP 0.067, KI 0.20, integral limit ±0.20, feedforward from T4.3. Files: [src/pid.cpp](../src/pid.cpp), [src/control.cpp](../src/control.cpp), [src/main.cpp](../src/main.cpp), [src/config.h](../src/config.h) |
| Procedure | `7` from standstill, twice; `6` at 4 and 8 rad/s; at 4 rad/s, left wheel braked by hand for ~3 s and released |
| Result | Steady state ✅. Step ❌ 25-28% overshoot (from 34-35%). Brake ❌ duty up to 0.62 (from 0.87), but peak 5.89 rad/s (+47%) and ~1.6 s back to the target: the integral stays at its limit and empties slowly with the lower KI |

```
  L  rise 0.08 s  overshoot 26.6%  steady  +4.01 rad/s (error +0.01)  FAIL
  R  rise 0.08 s  overshoot 24.8%  steady  +4.01 rad/s (error +0.01)  FAIL
  L  rise 0.08 s  overshoot 27.5%  steady  +4.00 rad/s (error +0.00)  FAIL
  R  rise 0.08 s  overshoot 26.2%  steady  +4.00 rad/s (error -0.00)  FAIL
```

```
  L  mean  +4.00 rad/s  std 0.010 (0.3%)  from ticks  +4.00 (diff 0.0%)  target  +4.00 (error -0.00)  PASS
  R  mean  +4.00 rad/s  std 0.007 (0.2%)  from ticks  +4.00 (diff 0.0%)  target  +4.00 (error -0.00)  PASS
  L  mean  +8.00 rad/s  std 0.034 (0.4%)  from ticks  +8.00 (diff 0.0%)  target  +8.00 (error +0.00)  PASS
  R  mean  +8.01 rad/s  std 0.013 (0.2%)  from ticks  +8.01 (diff 0.0%)  target  +8.00 (error +0.01)  PASS
```

Hand brake on the left wheel: braked at the integral limit, released, peak, back within tolerance:

```
L  28811 ticks  w  +2.54 rad/s  target +4.00  duty +0.62  glitch 0 fast 0 | R  29478 ticks  w  +4.00 rad/s  target +4.00  duty +0.32  glitch 0 fast 0
L  29103 ticks  w  +5.89 rad/s  target +4.00  duty +0.37  glitch 0 fast 0 | R  29807 ticks  w  +4.04 rad/s  target +4.00  duty +0.32  glitch 0 fast 0
L  29424 ticks  w  +4.38 rad/s  target +4.00  duty +0.32  glitch 0 fast 0 | R  30071 ticks  w  +4.00 rad/s  target +4.00  duty +0.32  glitch 0 fast 0
L  29697 ticks  w  +4.06 rad/s  target +4.00  duty +0.31  glitch 0 fast 0 | R  30334 ticks  w  +4.00 rad/s  target +4.00  duty +0.32  glitch 0 fast 0
```

---

## T5: Command watchdog

| | |
|---|---|
| Firmware | commit `5c490b8` (block 5). Files: [src/control.cpp](../src/control.cpp) (watchdog in the control task), [src/main.cpp](../src/main.cpp) (`v` lines, key `w`), [src/config.h](../src/config.h) (`CMD_TIMEOUT_S` 0.5), [tools/cmd_sender.py](../tools/cmd_sender.py) |
| Procedure | **T5.1:** watchdog on, key `1` once. **T5.2:** `python3 tools/cmd_sender.py` (4 rad/s at 20 Hz) for ~20 s, then Ctrl+C. **T5.3:** key `w` (watchdog off), key `1`, wait, key `4` |
| Criterion | T5.1 and T5.2: the wheels stop by themselves within 0.5 s of the last target, with the watchdog message. T5.2: no stop while the script runs. T5.3: no stop until key `4` |
| Result | ✅ PASS on all three |

T5.1, a single target: the watchdog stops the wheels after 0.5 s (one line every 200 ms):

```
L      0 ticks  w  +0.00 rad/s  target +4.00  duty +0.61  glitch 0 fast 0 | R      0 ticks  w  +0.00 rad/s  target +4.00  duty +0.62  glitch 0 fast 0
L     79 ticks  w  +5.06 rad/s  target +4.00  duty +0.28  glitch 0 fast 0 | R     78 ticks  w  +5.04 rad/s  target +4.00  duty +0.29  glitch 0 fast 0
L    150 ticks  w  +4.25 rad/s  target +4.00  duty +0.32  glitch 0 fast 0 | R    149 ticks  w  +4.25 rad/s  target +4.00  duty +0.33  glitch 0 fast 0
watchdog: no target for 0.50 s, motors off
L    204 ticks  w  +2.32 rad/s  target +0.00  duty +0.00  glitch 0 fast 0 | R    203 ticks  w  +2.33 rad/s  target +0.00  duty +0.00  glitch 0 fast 0
L    205 ticks  w  +0.07 rad/s  target +0.00  duty +0.00  glitch 0 fast 0 | R    204 ticks  w  +0.07 rad/s  target +0.00  duty +0.00  glitch 0 fast 0
```

T5.2, targets from the script: about 7000 ticks (~21 s) at 4.00 rad/s with no stop, then Ctrl+C. The board restarted when the script opened the port (first line):

```
rst:0x1 (POWERON_RESET),boot:0x13 (SPI_FAST_FLASH_BOOT)
...
L   7056 ticks  w  +3.99 rad/s  target +4.00  duty +0.31  glitch 0 fast 0 | R   7052 ticks  w  +4.02 rad/s  target +4.00  duty +0.32  glitch 0 fast 0
^C
--- stopped sending; the watchdog should stop the wheels (printing for 3 s more)
L   7122 ticks  w  +4.01 rad/s  target +4.00  duty +0.30  glitch 0 fast 0 | R   7118 ticks  w  +4.01 rad/s  target +4.00  duty +0.32  glitch 0 fast 0
L   7187 ticks  w  +4.02 rad/s  target +4.00  duty +0.30  glitch 0 fast 0 | R   7184 ticks  w  +4.00 rad/s  target +4.00  duty +0.32  glitch 0 fast 0
watchdog: no target for 0.50 s, motors off
L   7253 ticks  w  +4.00 rad/s  target +0.00  duty +0.00  glitch 0 fast 0 | R   7250 ticks  w  +4.01 rad/s  target +0.00  duty +0.00  glitch 0 fast 0
L   7271 ticks  w  +0.40 rad/s  target +0.00  duty +0.00  glitch 0 fast 0 | R   7267 ticks  w  +0.46 rad/s  target +0.00  duty +0.00  glitch 0 fast 0
```

T5.3, watchdog off: the wheels run ~3.4 s on a single key press, until key `4` (no watchdog message):

```
watchdog off
L     63 ticks  w  +5.15 rad/s  target +4.00  duty +0.28  glitch 0 fast 0 | R     62 ticks  w  +5.13 rad/s  target +4.00  duty +0.30  glitch 0 fast 0
L   1134 ticks  w  +3.99 rad/s  target +4.00  duty +0.31  glitch 0 fast 0 | R   1131 ticks  w  +3.99 rad/s  target +4.00  duty +0.32  glitch 0 fast 0
L   1189 ticks  w  +2.60 rad/s  target +0.00  duty +0.00  glitch 0 fast 0 | R   1187 ticks  w  +2.58 rad/s  target +0.00  duty +0.00  glitch 0 fast 0
```
