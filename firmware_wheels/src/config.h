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

// ---- Motor pins (L298N, ENA/ENB jumpers removed) ----
constexpr int PIN_ENA = 14;  // left PWM
constexpr int PIN_IN1 = 27;  // left direction
constexpr int PIN_IN2 = 26;
constexpr int PIN_IN3 = 25;  // right direction
constexpr int PIN_IN4 = 33;
constexpr int PIN_ENB = 32;  // right PWM

// Swaps the two direction inputs of a motor, so that a positive duty always
// moves the robot forward. The left motor turns backwards with IN1 = HIGH.
constexpr bool MOTOR_LEFT_INVERTED = true;
constexpr bool MOTOR_RIGHT_INVERTED = false;

// ---- Motor PWM (LEDC) ----
// The L298N is a slow bipolar bridge: a low frequency keeps its losses down.
constexpr uint32_t MOTOR_PWM_FREQ_HZ = 1000;
constexpr uint8_t MOTOR_PWM_RESOLUTION_BITS = 10;
constexpr uint8_t MOTOR_PWM_CHANNEL_LEFT = 0;
constexpr uint8_t MOTOR_PWM_CHANNEL_RIGHT = 1;

// Duty 0: false = coast (bridge off, the wheel spins freely),
// true = brake (both direction inputs LOW, enable HIGH).
constexpr bool MOTOR_BRAKE_ON_STOP = false;

// A motor is reversed only after its encoder has seen no edge for this long:
// the encoder cannot read the direction, so edges while the wheel is still
// coasting must be counted with the old one.
constexpr float REVERSE_WAIT_S = 0.05f;

// ---- Test keys (block 2) ----
constexpr float TEST_DUTY = 0.5f;        // duty used by keys 1-3
constexpr float TEST_DUTY_STEP = 0.05f;  // step of keys + and -, to find the dead zone

// ---- Serial ----
constexpr unsigned long SERIAL_BAUD = 115200;
constexpr unsigned long DEBUG_PERIOD_MS = 200;
