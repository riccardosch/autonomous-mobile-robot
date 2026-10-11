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

// ---- Control task ----
constexpr float CONTROL_PERIOD_S = 0.02f;  // 50 Hz, same rate as the future JointState
constexpr int CONTROL_TASK_CORE = 0;       // Arduino loop() and later micro-ROS run on core 1
constexpr int CONTROL_TASK_PRIORITY = 5;   // above loop() (1)
constexpr uint32_t CONTROL_TASK_STACK = 4096;

// ---- Speed estimate ----
// Time constant of the first-order low-pass filter on the measured speed.
constexpr float VELOCITY_FILTER_TAU_S = 0.04f;
// With no encoder edge for this long the wheel is considered stopped:
// the slowest measurable speed is one tick in this time, ~0.12 rad/s.
constexpr float VELOCITY_ZERO_TIMEOUT_S = 0.1f;

// ---- Speed controller ----
// PI with feedforward, one per wheel. KP from lambda tuning on the step test of
// block 3 (K ~15 rad/s per unit duty, time constant 0.10 s): KP = 1 / K.
// KI lowered from KP / time constant (0.67) to 0.20 and the integral limited:
// with the measured feedforward the integral only corrects small errors, and
// a large one caused 34% overshoot (test T4, wheels lifted). To be retuned
// with the robot on the ground (keys p/o and i/u change them at run time).
constexpr float PID_KP = 0.067f;     // duty per rad/s of error
constexpr float PID_KI = 0.20f;      // duty per rad of integrated error
constexpr float PID_I_MAX = 0.20f;   // |integral term| limit, in duty; also covers a ~20% lower supply

// Feedforward: duty = FF_OFFSET + |target| / FF_GAIN, with the target's sign.
// Straight-line fit of the steady closed-loop duty at 2, 4, 6 and 8 rad/s
// (wheels lifted, 14.7 V supply); the fit is within 0.02 duty of every point.
constexpr float FF_OFFSET_LEFT = 0.14f;    // duty
constexpr float FF_GAIN_LEFT = 21.6f;      // rad/s per unit duty
constexpr float FF_OFFSET_RIGHT = 0.125f;  // duty
constexpr float FF_GAIN_RIGHT = 19.0f;     // rad/s per unit duty

constexpr float SPEED_MAX = 8.0f;  // rad/s, targets are clamped to +-SPEED_MAX

// ---- Test keys ----
constexpr float TEST_DUTY = 0.5f;         // open-loop duty of the step test (key 5)
constexpr float TEST_SPEED = 4.0f;        // rad/s, target of keys 1-3 and of key 7
constexpr float TEST_SPEED_STEP = 1.0f;   // rad/s, step of keys + and -
constexpr float TUNE_FACTOR = 1.2f;       // keys p/o and i/u multiply or divide KP and KI by this

// Step test (key 5): duty TEST_DUTY from standstill for this long. The steady
// speed is the mean over the last STEP_TEST_SETTLED_S.
constexpr float STEP_TEST_DURATION_S = 2.0f;
constexpr float STEP_TEST_SETTLED_S = 0.5f;

// Stability test (key 6): filtered speed at constant duty over this long.
// Pass if std / mean <= STABILITY_MAX_STD_RATIO and the mean differs from the
// mean computed from the tick counts by at most STABILITY_MAX_MEAN_ERROR.
constexpr float STABILITY_TEST_DURATION_S = 5.0f;  // also the record buffer size
constexpr float STABILITY_MAX_STD_RATIO = 0.03f;
constexpr float STABILITY_MAX_MEAN_ERROR = 0.01f;

// Closed-loop speed tolerance: |mean - target| <= max(ratio * |target|, abs).
constexpr float SPEED_MAX_ERROR_RATIO = 0.03f;
constexpr float SPEED_MAX_ERROR_ABS = 0.1f;  // rad/s

// Closed-loop step test (key 7): target TEST_SPEED from standstill for this
// long. Pass if the 10-90% rise time and the overshoot are within these limits
// and the steady speed is within the tolerance above.
constexpr float CL_STEP_TEST_DURATION_S = 2.0f;
constexpr float CL_STEP_MAX_RISE_S = 0.3f;
constexpr float CL_STEP_MAX_OVERSHOOT = 0.10f;

// ---- Serial ----
constexpr unsigned long SERIAL_BAUD = 115200;
constexpr unsigned long DEBUG_PERIOD_MS = 200;
