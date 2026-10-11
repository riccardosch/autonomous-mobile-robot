# Wheel firmware test record

Every test run on the wheel firmware, with the code it was run on, the procedure, the pass criterion and the serial output lines that show the result. All tests so far were run with the wheels lifted off the ground, the ESP32 on `/dev/ttyUSB0` and the PlatformIO serial monitor (see [../README.md](../README.md#build-and-flash)).

| ID | Date | Block | What it checks | Result |
|---|---|---|---|---|
| [T1](#t1-encoder-counts-by-hand) | 2026-10-11 | 1 | Encoder transitions per wheel revolution | ✅ 516.6 /rev |
| [T2](#t2-motor-directions-and-dead-zone) | 2026-10-11 | 2 | Motor directions, tick signs, reversal, dead zone | ✅ |
| [T3](#t3-speed-estimate) | 2026-10-11 | 3 | Speed estimate stability, step response | ✅ PASS |

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
