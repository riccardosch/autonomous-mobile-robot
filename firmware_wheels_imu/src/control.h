#pragma once

#include "encoders.h"
#include "imu.h"
#include "pid.h"

struct WheelState {
  float position;      // rad since start, + = forward
  float velocity;      // rad/s, filtered
  float velocity_raw;  // rad/s, before the filter
  float target;        // rad/s, 0 in open loop
  float duty;          // duty applied to the bridge
};

struct ImuState {
  bool ok;            // false if the IMU is missing or the last read failed
  ImuSample sample;   // robot frame, gyro bias removed
  float yaw;          // rad, integral of gyro z since start (drifts, for tests)
  ImuCal calibration;
  uint32_t calibration_restarts;
  float gyro_bias[3];  // rad/s
  uint32_t read_errors;
};

// One sample per control period, recorded for the tests.
struct RecordedSample {
  float velocity_raw[WHEEL_COUNT];
  float velocity[WHEEL_COUNT];
  ImuSample imu;
};

constexpr int RECORD_MAX_SAMPLES = 256;

// Starts the fixed-period control task (CONTROL_PERIOD_S, core CONTROL_TASK_CORE).
// The task is the only one that drives the motors.
void control_begin();

// Closed loop: target speed of each wheel in rad/s, clamped to +-SPEED_MAX.
// A target of 0 turns the motor off. Each call also feeds the watchdog: with
// no call for CMD_TIMEOUT_S the targets go to 0.
void control_set_target(float left, float right);

// Open loop: duty of each wheel in [-1, 1], for the tests.
void control_set_duty(float left, float right);

// Command watchdog, on at boot if CMD_WATCHDOG_ON_AT_BOOT.
void control_set_watchdog(bool on);
bool control_watchdog_on();
// True from a watchdog stop until the next target.
bool control_watchdog_tripped();

// Speed controller gains, shared by both wheels. Start from PID_KP, PID_KI and PID_I_MAX.
void control_set_gains(const PiGains &g);
PiGains control_get_gains();

// Consistent copy of the latest state of both wheels.
void control_get_state(WheelState out[WHEEL_COUNT]);

// Latest IMU state, and a request to calibrate the gyro again.
void control_get_imu(ImuState &out);
void control_imu_calibrate();

// Longest run of one control period since the last call, in microseconds.
uint32_t control_max_run_us();

// Records the next `samples` periods (at most RECORD_MAX_SAMPLES).
void control_record_start(int samples);
void control_record_stop();
bool control_record_done();

// Valid once control_record_done() is true, until the next start.
const RecordedSample *control_record_samples();

// Mean speed over the recording from the tick counts alone, rad/s.
float control_record_tick_speed(Wheel wheel);
