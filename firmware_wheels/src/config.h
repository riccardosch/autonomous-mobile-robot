#pragma once

#include <cstdint>

// ---- Encoder pins (ESP32 DevKit) ----
// Single-channel Hall encoders, not quadrature: the direction cannot be read,
// it is taken from the commanded motor direction.
constexpr int PIN_ENC_LEFT = 18;
constexpr int PIN_ENC_RIGHT = 19;

// ---- Encoder ----
// Transitions (both edges) per wheel revolution. Measured by hand on 2026-10-11:
// mean of 4 runs of 5 revolutions (2 per wheel), about ±1%. Both wheels give the
// same value. The iRobot Create figure is 508.8. To refine with a drive over a
// known distance.
constexpr float ENCODER_TICKS_PER_REV = 516.6f;

// Edges closer than this to the previous accepted edge are counted as glitches
// and dropped. At the maximum wheel speed (8 rad/s) edges are ~1.5 ms apart.
constexpr uint32_t ENCODER_GLITCH_US = 100;

// Accepted edges closer than this to the previous one are counted as "fast":
// 500 us is ~25 rad/s, three times the maximum wheel speed, so a fast edge is
// almost certainly a double edge from a noisy signal, not real motion.
constexpr uint32_t ENCODER_FAST_US = 500;

// ---- Serial ----
constexpr unsigned long SERIAL_BAUD = 115200;
constexpr unsigned long DEBUG_PERIOD_MS = 200;
