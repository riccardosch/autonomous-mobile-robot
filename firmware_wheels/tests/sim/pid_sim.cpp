// PC simulation of the wheel speed controller (test T4).
//
// Runs the firmware's own PI (src/pid.cpp) with the gains and feedforward of
// src/config.h on a model of the left wheel built from the hardware tests:
//   - steady speed vs duty: closed-loop duties at 2-8 rad/s and open-loop
//     speeds at 0.7 and 1.0 duty (wheels lifted, 14.7 V supply);
//   - first-order response with time constant 0.10 s (step test T3.2);
//   - static friction: from standstill the wheel starts only at duty 0.35 (T2);
//   - same speed filter and control period as the firmware.
// With the settings of the first version it predicts 31% overshoot on the
// 0 -> 4 rad/s step; the hardware gave 34-35%. It overstates the speed peak
// after a hand brake (93% against 26% measured): the simulated hand is harder.
//
// Build and run from firmware_wheels/:
//   g++ -O2 -Isrc tests/sim/pid_sim.cpp src/pid.cpp -o /tmp/pid_sim
//   /tmp/pid_sim                          # settings from config.h
//   /tmp/pid_sim KP KI I_MAX FF_OFFSET FF_GAIN
//   /tmp/pid_sim 0.067 0.67 1 0 14.8      # first version

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "config.h"
#include "pid.h"

namespace {

constexpr float PLANT_TAU_S = 0.10f;
constexpr float BREAKAWAY_DUTY = 0.35f;
constexpr float PLANT_STEP_S = 0.001f;
constexpr float BRAKE_SPEED = 2.5f;  // rad/s the hand holds the wheel at
constexpr float BRAKE_S = 2.0f;

// Steady speed (rad/s) for a duty, left wheel, linear between measured points.
float steady_speed(float duty) {
  const float u[] = {0.12f, 0.24f, 0.315f, 0.40f, 0.52f, 0.70f, 1.00f};
  const float w[] = {0.0f, 2.0f, 4.0f, 6.0f, 8.0f, 9.4f, 11.8f};
  if (duty <= u[0]) return 0.0f;
  for (int i = 1; i < 7; i++) {
    if (duty <= u[i]) return w[i - 1] + (w[i] - w[i - 1]) * (duty - u[i - 1]) / (u[i] - u[i - 1]);
  }
  return w[6];
}

float ff_offset = FF_OFFSET_LEFT;
float ff_gain = FF_GAIN_LEFT;

struct Sim {
  PiGains gains;
  PiState pi = {0.0f};
  float speed = 0.0f;     // true wheel speed
  float measured = 0.0f;  // filtered, as in the firmware

  // One control period; `held` = a hand holds the wheel at BRAKE_SPEED.
  void step(float target, bool held) {
    const float ff = ff_offset + target / ff_gain;
    const float duty = pi_update(pi, gains, target, measured, ff, CONTROL_PERIOD_S, 0.0f, 1.0f, false);
    for (float t = 0.0f; t < CONTROL_PERIOD_S; t += PLANT_STEP_S) {
      const float goal = held ? BRAKE_SPEED : steady_speed(duty);
      const bool stuck = !held && speed < 0.05f && duty < BREAKAWAY_DUTY;
      speed = stuck ? 0.0f : speed + (goal - speed) / PLANT_TAU_S * PLANT_STEP_S;
    }
    measured += CONTROL_PERIOD_S / (VELOCITY_FILTER_TAU_S + CONTROL_PERIOD_S) * (speed - measured);
  }
};

int periods(float s) { return static_cast<int>(lroundf(s / CONTROL_PERIOD_S)); }

// Step from `from` to `to` rad/s; then a hand brake and release.
void run(const PiGains &g, float from, float to) {
  Sim sim{g};
  for (int i = 0; from > 0.0f && i < periods(2.0f); i++) sim.step(from, false);
  float peak = 0.0f;
  int i10 = -1, i90 = -1;
  const float span = to - from;
  for (int i = 0; i < periods(2.0f); i++) {
    sim.step(to, false);
    peak = std::max(peak, sim.measured);
    if (i10 < 0 && sim.measured >= from + 0.1f * span) i10 = i;
    if (i90 < 0 && sim.measured >= from + 0.9f * span) i90 = i;
  }
  const float error = sim.measured - to;
  float brake_peak = 0.0f;
  int settled = 0;  // last period outside the tolerance after the release
  for (int i = 0; i < periods(BRAKE_S + 3.0f); i++) {
    const bool held = i < periods(BRAKE_S);
    sim.step(to, held);
    if (held) continue;
    brake_peak = std::max(brake_peak, sim.measured);
    const float tol = std::max(SPEED_MAX_ERROR_RATIO * to, SPEED_MAX_ERROR_ABS);
    if (fabsf(sim.measured - to) > tol) settled = i - periods(BRAKE_S) + 1;
  }
  printf("  %.0f -> %.0f rad/s: rise %.2f s  overshoot %5.1f%%  error %+.2f | after brake: peak %+5.1f%%  settled in %.2f s\n",
         from, to, (i90 - i10) * CONTROL_PERIOD_S, 100.0f * (peak - to) / to, error,
         100.0f * (brake_peak - to) / to, settled * CONTROL_PERIOD_S);
}

}  // namespace

int main(int argc, char **argv) {
  PiGains g = {PID_KP, PID_KI, PID_I_MAX};
  if (argc == 6) {
    g = {strtof(argv[1], nullptr), strtof(argv[2], nullptr), strtof(argv[3], nullptr)};
    ff_offset = strtof(argv[4], nullptr);
    ff_gain = strtof(argv[5], nullptr);
  }
  printf("KP %.3f  KI %.3f  integral limit %.2f  feedforward %.3f + target / %.1f\n", g.kp, g.ki,
         g.i_max, ff_offset, ff_gain);
  run(g, 0.0f, 2.0f);
  run(g, 0.0f, 4.0f);
  run(g, 0.0f, 8.0f);
  run(g, 2.0f, 6.0f);
  run(g, 4.0f, 8.0f);
  return 0;
}
