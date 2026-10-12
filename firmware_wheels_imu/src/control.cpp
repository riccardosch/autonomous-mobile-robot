#include "control.h"

#include <Arduino.h>

#include "config.h"
#include "motors.h"
#include "pid.h"
#include "velocity.h"

namespace {

constexpr float RAD_PER_TICK = 2.0f * PI / ENCODER_TICKS_PER_REV;
constexpr uint32_t PERIOD_MS = static_cast<uint32_t>(CONTROL_PERIOD_S * 1000.0f + 0.5f);
constexpr uint32_t CMD_TIMEOUT_MS = static_cast<uint32_t>(CMD_TIMEOUT_S * 1000.0f + 0.5f);

static_assert(STABILITY_TEST_DURATION_S / CONTROL_PERIOD_S <= RECORD_MAX_SAMPLES,
              "stability test longer than the record buffer");
static_assert(STEP_TEST_DURATION_S / CONTROL_PERIOD_S <= RECORD_MAX_SAMPLES,
              "step test longer than the record buffer");
static_assert(CL_STEP_TEST_DURATION_S / CONTROL_PERIOD_S <= RECORD_MAX_SAMPLES,
              "closed-loop step test longer than the record buffer");
static_assert(IMU_TEST_DURATION_S / CONTROL_PERIOD_S <= RECORD_MAX_SAMPLES,
              "IMU test longer than the record buffer");

const float FF_OFFSET[WHEEL_COUNT] = {FF_OFFSET_LEFT, FF_OFFSET_RIGHT};
const float FF_GAIN[WHEEL_COUNT] = {FF_GAIN_LEFT, FF_GAIN_RIGHT};

// Used by the control task only.
PiState pi[WHEEL_COUNT] = {};

// Shared between the control task (core 0) and loop() (core 1).
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
bool closed_loop = false;
bool watchdog_on = CMD_WATCHDOG_ON_AT_BOOT;
bool watchdog_tripped = false;
uint32_t last_target_ms = 0;
PiGains gains = {PID_KP, PID_KI, PID_I_MAX};
float command[WHEEL_COUNT] = {};  // rad/s in closed loop, duty in open loop
WheelState state[WHEEL_COUNT] = {};
ImuState imu_state = {};
bool imu_cal_request = false;
uint32_t max_run_us = 0;
RecordedSample samples[RECORD_MAX_SAMPLES];
int record_target = 0;
int record_count = 0;
int32_t record_start_ticks[WHEEL_COUNT] = {};
int32_t record_end_ticks[WHEEL_COUNT] = {};

// Duty for one wheel. The output never opposes the target: the encoder cannot
// read the direction, so the controller only drives towards the target and
// lets the wheel coast if it is too fast.
float speed_control(int w, float target, float measured, const PiGains &g) {
  if (target == 0.0f) {
    pi_reset(pi[w]);
    return 0.0f;
  }
  const float ff = copysignf(FF_OFFSET[w] + fabsf(target) / FF_GAIN[w], target);
  const MotorStatus m = motors_status(static_cast<Wheel>(w));
  const bool reversing = m.applied != m.requested;  // waiting for the wheel to stop
  const float out_min = target > 0.0f ? 0.0f : -1.0f;
  const float out_max = target > 0.0f ? 1.0f : 0.0f;
  return pi_update(pi[w], g, target, measured, ff, CONTROL_PERIOD_S, out_min, out_max, reversing);
}

void control_task(void *) {
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(PERIOD_MS));
    const uint32_t start_us = micros();
    EncoderSnapshot enc[WHEEL_COUNT];
    encoders_read(enc);
    VelocityEstimate v[WHEEL_COUNT];
    velocity_update(enc, micros(), v);

    const uint32_t now_ms = millis();
    portENTER_CRITICAL(&mux);
    const bool moving = command[WHEEL_LEFT] != 0.0f || command[WHEEL_RIGHT] != 0.0f;
    if (closed_loop && watchdog_on && moving && now_ms - last_target_ms > CMD_TIMEOUT_MS) {
      for (int w = 0; w < WHEEL_COUNT; w++) command[w] = 0.0f;
      watchdog_tripped = true;
    }
    const bool closed = closed_loop;
    const PiGains g = gains;
    const bool calibrate = imu_cal_request;
    imu_cal_request = false;
    float cmd[WHEEL_COUNT];
    for (int w = 0; w < WHEEL_COUNT; w++) cmd[w] = command[w];
    portEXIT_CRITICAL(&mux);

    float duty[WHEEL_COUNT];
    for (int w = 0; w < WHEEL_COUNT; w++) {
      if (closed) {
        duty[w] = speed_control(w, cmd[w], v[w].filtered, g);
      } else {
        pi_reset(pi[w]);
        duty[w] = cmd[w];
      }
      motors_set(static_cast<Wheel>(w), duty[w]);
    }

    if (calibrate) imu_start_calibration();
    ImuSample imu = {};
    const bool imu_ok = imu_read(imu);

    portENTER_CRITICAL(&mux);
    for (int w = 0; w < WHEEL_COUNT; w++) {
      state[w] = {enc[w].ticks * RAD_PER_TICK, v[w].filtered, v[w].raw, closed ? cmd[w] : 0.0f,
                  motors_status(static_cast<Wheel>(w)).applied};
    }
    imu_state.ok = imu_ok;
    if (imu_ok) {
      imu_state.sample = imu;
      imu_state.yaw += imu.gyro[2] * CONTROL_PERIOD_S;
    } else {
      imu_state.read_errors++;
    }
    imu_state.calibration = imu_calibration_state();
    imu_state.calibration_restarts = imu_calibration_restarts();
    imu_gyro_bias(imu_state.gyro_bias);
    if (record_count < record_target) {
      RecordedSample &s = samples[record_count];
      s.imu = imu;
      for (int w = 0; w < WHEEL_COUNT; w++) {
        if (record_count == 0) record_start_ticks[w] = enc[w].ticks;
        record_end_ticks[w] = enc[w].ticks;
        s.velocity_raw[w] = v[w].raw;
        s.velocity[w] = v[w].filtered;
      }
      record_count++;
    }
    max_run_us = max(max_run_us, static_cast<uint32_t>(micros() - start_us));
    portEXIT_CRITICAL(&mux);
  }
}

}  // namespace

void control_begin() {
  xTaskCreatePinnedToCore(control_task, "control", CONTROL_TASK_STACK, nullptr,
                          CONTROL_TASK_PRIORITY, nullptr, CONTROL_TASK_CORE);
}

void control_set_target(float left, float right) {
  portENTER_CRITICAL(&mux);
  closed_loop = true;
  command[WHEEL_LEFT] = constrain(left, -SPEED_MAX, SPEED_MAX);
  command[WHEEL_RIGHT] = constrain(right, -SPEED_MAX, SPEED_MAX);
  last_target_ms = millis();
  watchdog_tripped = false;
  portEXIT_CRITICAL(&mux);
}

void control_set_duty(float left, float right) {
  portENTER_CRITICAL(&mux);
  closed_loop = false;
  command[WHEEL_LEFT] = constrain(left, -1.0f, 1.0f);
  command[WHEEL_RIGHT] = constrain(right, -1.0f, 1.0f);
  portEXIT_CRITICAL(&mux);
}

void control_set_watchdog(bool on) {
  portENTER_CRITICAL(&mux);
  watchdog_on = on;
  last_target_ms = millis();  // a full timeout before it can trip
  portEXIT_CRITICAL(&mux);
}

bool control_watchdog_on() {
  portENTER_CRITICAL(&mux);
  const bool on = watchdog_on;
  portEXIT_CRITICAL(&mux);
  return on;
}

bool control_watchdog_tripped() {
  portENTER_CRITICAL(&mux);
  const bool tripped = watchdog_tripped;
  portEXIT_CRITICAL(&mux);
  return tripped;
}

void control_set_gains(float kp, float ki) {
  portENTER_CRITICAL(&mux);
  gains.kp = kp;
  gains.ki = ki;
  portEXIT_CRITICAL(&mux);
}

void control_get_gains(float &kp, float &ki) {
  portENTER_CRITICAL(&mux);
  kp = gains.kp;
  ki = gains.ki;
  portEXIT_CRITICAL(&mux);
}

void control_get_state(WheelState out[WHEEL_COUNT]) {
  portENTER_CRITICAL(&mux);
  for (int w = 0; w < WHEEL_COUNT; w++) out[w] = state[w];
  portEXIT_CRITICAL(&mux);
}

void control_get_imu(ImuState &out) {
  portENTER_CRITICAL(&mux);
  out = imu_state;
  portEXIT_CRITICAL(&mux);
}

void control_imu_calibrate() {
  portENTER_CRITICAL(&mux);
  imu_cal_request = true;
  portEXIT_CRITICAL(&mux);
}

uint32_t control_max_run_us() {
  portENTER_CRITICAL(&mux);
  const uint32_t us = max_run_us;
  max_run_us = 0;
  portEXIT_CRITICAL(&mux);
  return us;
}

void control_record_start(int n) {
  portENTER_CRITICAL(&mux);
  record_count = 0;
  record_target = constrain(n, 0, RECORD_MAX_SAMPLES);
  portEXIT_CRITICAL(&mux);
}

void control_record_stop() {
  portENTER_CRITICAL(&mux);
  record_target = 0;
  record_count = 0;
  portEXIT_CRITICAL(&mux);
}

bool control_record_done() {
  portENTER_CRITICAL(&mux);
  const bool done = record_target > 0 && record_count == record_target;
  portEXIT_CRITICAL(&mux);
  return done;
}

const RecordedSample *control_record_samples() { return samples; }

float control_record_tick_speed(Wheel wheel) {
  portENTER_CRITICAL(&mux);
  const int32_t dticks = record_end_ticks[wheel] - record_start_ticks[wheel];
  const int periods = record_count - 1;  // the ticks are taken at the first and last sample
  portEXIT_CRITICAL(&mux);
  return periods > 0 ? dticks * RAD_PER_TICK / (periods * CONTROL_PERIOD_S) : 0.0f;
}
