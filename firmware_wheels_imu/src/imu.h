#pragma once

#include <cstdint>

// BMI160 accelerometer and gyroscope on I2C.

constexpr float STANDARD_GRAVITY = 9.80665f;  // m/s^2

struct ImuInfo {
  bool found;       // chip id read at `address` on these pins
  bool configured;  // every configuration register read back as written
  uint8_t address;
  int sda;
  int scl;
  // Read back after the configuration, for diagnostics.
  uint8_t acc_conf, acc_range, gyr_conf, gyr_range, pmu_status, err_reg;
};

// One sample in the robot frame (x forward, y left, z up), gyro bias removed.
struct ImuSample {
  float accel[3];  // m/s^2
  float gyro[3];   // rad/s
};

enum class ImuCal { RUNNING, DONE };

// Looks for the BMI160 on the configured pins (then on the pins swapped) at
// both addresses, and configures it. The IMU is used only if `configured`.
// Call once, before the control task.
ImuInfo imu_begin();

// The functions below are for the control task only; other tasks go through
// control.h.

// Reads one sample. False if the IMU is missing or the read fails.
bool imu_read(ImuSample &out);

// Starts the gyro bias calibration (also started by imu_begin()). It runs on
// the next imu_read() calls, IMU_CAL_S long.
void imu_start_calibration();
ImuCal imu_calibration_state();
uint32_t imu_calibration_restarts();  // times the robot moved during a calibration
void imu_gyro_bias(float out[3]);     // rad/s, robot frame
