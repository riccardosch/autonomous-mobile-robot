#pragma once

#include <cstdint>

enum Wheel { WHEEL_LEFT = 0, WHEEL_RIGHT = 1, WHEEL_COUNT = 2 };

// Consistent copy of one encoder's state, taken with interrupts masked.
struct EncoderSnapshot {
  int32_t ticks;          // accepted transitions since start or last reset
  uint32_t last_edge_us;  // micros() of the last accepted transition
  uint32_t glitches;      // transitions dropped by the glitch filter
  uint32_t fast;          // accepted transitions closer than ENCODER_FAST_US
};

// Configures the pins and attaches the interrupts (CHANGE, both edges).
void encoders_begin();

// Copies the state of both encoders, indexed by Wheel.
void encoders_read(EncoderSnapshot out[WHEEL_COUNT]);

// Sets ticks, glitch and fast counters back to zero.
void encoders_reset();
