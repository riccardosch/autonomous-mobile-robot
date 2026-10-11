#include "pid.h"

void pi_reset(PiState &s) { s.integral = 0.0f; }

float pi_update(PiState &s, const PiGains &g, float target, float measured, float feedforward,
                float dt, float out_min, float out_max, bool hold) {
  const float error = target - measured;
  float candidate = s.integral + g.ki * error * dt;
  if (candidate > g.i_max) candidate = g.i_max;
  if (candidate < -g.i_max) candidate = -g.i_max;
  const float out = feedforward + g.kp * error + candidate;
  const bool winding_up = (out > out_max && error > 0.0f) || (out < out_min && error < 0.0f);
  if (!hold && !winding_up) s.integral = candidate;

  const float result = feedforward + g.kp * error + s.integral;
  if (result > out_max) return out_max;
  if (result < out_min) return out_min;
  return result;
}
