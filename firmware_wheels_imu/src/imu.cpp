#include "imu.h"

#include <Arduino.h>
#include <Wire.h>

#include "config.h"

namespace {

// BMI160 registers and values (Bosch BMI160 datasheet).
constexpr uint8_t REG_CHIP_ID = 0x00;
constexpr uint8_t REG_ERR = 0x02;
constexpr uint8_t REG_PMU_STATUS = 0x03;
constexpr uint8_t REG_DATA_GYR_X = 0x0C;  // gyro x, y, z then accel x, y, z, int16 little-endian
constexpr uint8_t REG_ACC_CONF = 0x40;
constexpr uint8_t REG_ACC_RANGE = 0x41;
constexpr uint8_t REG_GYR_CONF = 0x42;
constexpr uint8_t REG_GYR_RANGE = 0x43;
constexpr uint8_t REG_CMD = 0x7E;
constexpr uint8_t CHIP_ID = 0xD1;
constexpr uint8_t CMD_SOFT_RESET = 0xB6;
constexpr uint8_t CMD_ACC_NORMAL = 0x11;
constexpr uint8_t CMD_GYR_NORMAL = 0x15;
constexpr uint8_t PMU_ACC_GYR_NORMAL = 0x14;  // both in normal mode
constexpr uint8_t BWP_NORMAL = 0x02;          // filter bandwidth: normal mode
constexpr uint32_t SOFT_RESET_MS = 10;
// In suspend mode (after the reset) the BMI160 needs 450 us between two
// register writes, or it ignores the second one.
constexpr uint32_t WRITE_GAP_US = 1000;
constexpr uint32_t ACC_STARTUP_MS = 5;
constexpr uint32_t GYR_STARTUP_MS = 80;

constexpr uint8_t odr_code(int hz) {
  return hz == 25 ? 0x06 : hz == 50 ? 0x07 : hz == 100 ? 0x08 : hz == 200 ? 0x09 : 0xFF;
}
constexpr uint8_t acc_range_code(int g) {
  return g == 2 ? 0x03 : g == 4 ? 0x05 : g == 8 ? 0x08 : g == 16 ? 0x0C : 0xFF;
}
constexpr uint8_t gyr_range_code(int dps) {
  return dps == 2000 ? 0x00 : dps == 1000 ? 0x01 : dps == 500 ? 0x02 : dps == 250 ? 0x03
       : dps == 125 ? 0x04 : 0xFF;
}
static_assert(odr_code(IMU_ODR_HZ) != 0xFF, "unsupported IMU_ODR_HZ");
static_assert(acc_range_code(IMU_ACCEL_RANGE_G) != 0xFF, "unsupported IMU_ACCEL_RANGE_G");
static_assert(gyr_range_code(IMU_GYRO_RANGE_DPS) != 0xFF, "unsupported IMU_GYRO_RANGE_DPS");

constexpr float ACCEL_SCALE = IMU_ACCEL_RANGE_G * STANDARD_GRAVITY / 32768.0f;  // m/s^2 per LSB
constexpr float GYRO_SCALE = IMU_GYRO_RANGE_DPS * (PI / 180.0f) / 32768.0f;     // rad/s per LSB
constexpr int CAL_SAMPLES = static_cast<int>(IMU_CAL_S * IMU_ODR_HZ + 0.5f);

bool present = false;
uint8_t address = 0;

struct Calibration {
  ImuCal state;
  int count;
  float sum[3];
  float min[3];
  float max[3];
  uint32_t restarts;
};
Calibration cal = {};
float bias[3] = {};

bool write_reg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  const bool ok = Wire.endTransmission() == 0;
  delayMicroseconds(WRITE_GAP_US);
  return ok;
}

bool read_regs(uint8_t reg, uint8_t *buf, size_t n) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(address, static_cast<uint8_t>(n)) != n) return false;
  for (size_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

bool find_on_pins(int sda, int scl) {
  Wire.end();
  Wire.begin(sda, scl, I2C_FREQ_HZ);
  Wire.setTimeOut(I2C_TIMEOUT_MS);
  for (uint8_t a : {IMU_I2C_ADDR, IMU_I2C_ADDR_ALT}) {
    address = a;
    uint8_t id = 0;
    if (read_regs(REG_CHIP_ID, &id, 1) && id == CHIP_ID) return true;
  }
  return false;
}

uint8_t read_reg(uint8_t reg) {
  uint8_t value = 0;
  read_regs(reg, &value, 1);
  return value;
}

constexpr uint8_t ACC_CONF_VALUE = (BWP_NORMAL << 4) | odr_code(IMU_ODR_HZ);
constexpr uint8_t GYR_CONF_VALUE = (BWP_NORMAL << 4) | odr_code(IMU_ODR_HZ);

bool configure() {
  if (!write_reg(REG_CMD, CMD_SOFT_RESET)) return false;
  delay(SOFT_RESET_MS);
  const bool ok = write_reg(REG_ACC_CONF, ACC_CONF_VALUE) &&
                  write_reg(REG_ACC_RANGE, acc_range_code(IMU_ACCEL_RANGE_G)) &&
                  write_reg(REG_GYR_CONF, GYR_CONF_VALUE) &&
                  write_reg(REG_GYR_RANGE, gyr_range_code(IMU_GYRO_RANGE_DPS)) &&
                  write_reg(REG_CMD, CMD_ACC_NORMAL);
  if (!ok) return false;
  delay(ACC_STARTUP_MS);
  if (!write_reg(REG_CMD, CMD_GYR_NORMAL)) return false;
  delay(GYR_STARTUP_MS);
  return true;
}

// Reads the configuration back into `info`; true if it is what was written.
bool verify(ImuInfo &info) {
  info.acc_conf = read_reg(REG_ACC_CONF);
  info.acc_range = read_reg(REG_ACC_RANGE);
  info.gyr_conf = read_reg(REG_GYR_CONF);
  info.gyr_range = read_reg(REG_GYR_RANGE);
  info.pmu_status = read_reg(REG_PMU_STATUS);
  info.err_reg = read_reg(REG_ERR);
  return info.acc_conf == ACC_CONF_VALUE && info.acc_range == acc_range_code(IMU_ACCEL_RANGE_G) &&
         info.gyr_conf == GYR_CONF_VALUE && info.gyr_range == gyr_range_code(IMU_GYRO_RANGE_DPS) &&
         info.pmu_status == PMU_ACC_GYR_NORMAL;
}

// Sensor frame to robot frame.
void to_robot(const float in[3], float out[3]) {
  for (int i = 0; i < 3; i++) out[i] = IMU_SIGN_FOR_ROBOT[i] * in[IMU_AXIS_FOR_ROBOT[i]];
}

void calibrate(const float gyro[3]) {
  for (int i = 0; i < 3; i++) {
    cal.sum[i] += gyro[i];
    cal.min[i] = cal.count == 0 ? gyro[i] : min(cal.min[i], gyro[i]);
    cal.max[i] = cal.count == 0 ? gyro[i] : max(cal.max[i], gyro[i]);
  }
  if (++cal.count < CAL_SAMPLES) return;
  for (int i = 0; i < 3; i++) {
    if (cal.max[i] - cal.min[i] > IMU_CAL_MAX_SPREAD_RAD_S) {  // moved: start again
      cal.restarts++;
      cal.count = 0;
      for (int j = 0; j < 3; j++) cal.sum[j] = 0.0f;
      return;
    }
  }
  for (int i = 0; i < 3; i++) bias[i] = cal.sum[i] / cal.count;
  cal.state = ImuCal::DONE;
}

}  // namespace

ImuInfo imu_begin() {
  ImuInfo info = {};
  const int pins[2][2] = {{PIN_I2C_SDA, PIN_I2C_SCL}, {PIN_I2C_SCL, PIN_I2C_SDA}};
  for (const auto &p : pins) {
    if (!find_on_pins(p[0], p[1])) continue;
    info.found = true;
    info.address = address;
    info.sda = p[0];
    info.scl = p[1];
    info.configured = configure() && verify(info);
    present = info.configured;
    if (present) imu_start_calibration();
    return info;
  }
  info.sda = PIN_I2C_SDA;
  info.scl = PIN_I2C_SCL;
  return info;
}

bool imu_read(ImuSample &out) {
  if (!present) return false;
  uint8_t buf[12];
  if (!read_regs(REG_DATA_GYR_X, buf, sizeof(buf))) return false;
  float gyro[3], accel[3];
  for (int i = 0; i < 3; i++) {
    gyro[i] = static_cast<int16_t>(buf[2 * i] | (buf[2 * i + 1] << 8)) * GYRO_SCALE;
    accel[i] = static_cast<int16_t>(buf[6 + 2 * i] | (buf[7 + 2 * i] << 8)) * ACCEL_SCALE;
  }
  float gyro_robot[3];
  to_robot(gyro, gyro_robot);
  to_robot(accel, out.accel);
  if (cal.state == ImuCal::RUNNING) calibrate(gyro_robot);
  for (int i = 0; i < 3; i++) out.gyro[i] = gyro_robot[i] - bias[i];
  return true;
}

void imu_start_calibration() {
  const uint32_t restarts = cal.restarts;
  cal = {};
  cal.state = ImuCal::RUNNING;
  cal.restarts = restarts;
}

ImuCal imu_calibration_state() { return cal.state; }

uint32_t imu_calibration_restarts() { return cal.restarts; }

void imu_gyro_bias(float out[3]) {
  for (int i = 0; i < 3; i++) out[i] = bias[i];
}
