#pragma once

// PI speed controller with feedforward and clamping anti-windup.
// Plain C++, no Arduino dependency.

struct PiGains {
  float kp;     // output per unit of error
  float ki;     // output per unit of integrated error
  float i_max;  // limit of |integral term|, in output units
};

struct PiState {
  float integral;  // integral term, in output units
};

void pi_reset(PiState &s);

// One control step. The output is feedforward + P + I, clamped to
// [out_min, out_max]. The integral is limited to +-i_max, and not updated when
// `hold` is true or when the output is clamped and the error would push it
// further out.
float pi_update(PiState &s, const PiGains &g, float target, float measured, float feedforward,
                float dt, float out_min, float out_max, bool hold);
