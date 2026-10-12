#pragma once

#include <cstdint>

#include "encoders.h"

struct VelocityEstimate {
  float raw;       // rad/s, before the filter
  float filtered;  // rad/s, after the low-pass filter
};

// Wheel speed from the encoder counts and edge times. Call it once per
// control period, always from the same task: it keeps the previous snapshot.
void velocity_update(const EncoderSnapshot enc[WHEEL_COUNT], uint32_t now_us,
                     VelocityEstimate out[WHEEL_COUNT]);
