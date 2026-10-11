#pragma once

#include "encoders.h"

// Duty of one motor: requested by the caller, and actually applied to the
// bridge (0 while waiting for the wheel to stop before reversing).
struct MotorStatus {
  float requested;
  float applied;
};

// Configures the bridge pins and the PWM, with both motors off.
void motors_begin();

// Requests a duty in [-1, 1]: the sign is the direction (+ = robot forward),
// the magnitude the PWM. Values outside the range are clamped. A reversal is
// completed only once the wheel has stopped, so call it every control period.
void motors_set(Wheel wheel, float duty);

MotorStatus motors_status(Wheel wheel);
