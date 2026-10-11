// Wheel firmware for the ESP32 DevKit.
//
// Block 3: open-loop motor control from the serial monitor, with the wheel
// speed in rad/s measured by the control task every CONTROL_PERIOD_S.
// Single keys, no Enter needed:
//   1  both wheels forward at TEST_DUTY
//   2  both wheels backward at TEST_DUTY
//   3  left forward, right backward
//   4  stop both motors (also aborts a running test)
//   +  both forward, duty up by TEST_DUTY_STEP (to find the dead zone)
//   -  duty down by TEST_DUTY_STEP
//   5  step test: TEST_DUTY from standstill, prints steady speed, gain and time constant
//   6  stability test: with the wheels turning, prints mean and spread of the speed
//   z  zero the displayed tick and glitch counters

#include <Arduino.h>

#include "config.h"
#include "control.h"
#include "encoders.h"
#include "motors.h"

namespace {

enum class Test { NONE, STEP, STABILITY };

const char *const WHEEL_NAMES[WHEEL_COUNT] = {"L", "R"};

unsigned long last_print_ms = 0;
float step_duty = 0.0f;  // duty set by the + and - keys
Test running = Test::NONE;
EncoderSnapshot zero[WHEEL_COUNT] = {};  // counters at the last 'z'

int periods(float seconds) { return lroundf(seconds / CONTROL_PERIOD_S); }

void set_both(float left, float right) {
  motors_set(WHEEL_LEFT, left);
  motors_set(WHEEL_RIGHT, right);
}

bool wheels_stopped() {
  WheelState st[WHEEL_COUNT];
  control_get_state(st);
  for (int w = 0; w < WHEEL_COUNT; w++) {
    if (st[w].velocity_raw != 0.0f || motors_status(static_cast<Wheel>(w)).applied != 0.0f) return false;
  }
  return true;
}

bool wheels_driven() {
  for (int w = 0; w < WHEEL_COUNT; w++) {
    if (motors_status(static_cast<Wheel>(w)).applied == 0.0f) return false;
  }
  return true;
}

void start_step_test() {
  if (!wheels_stopped()) {
    Serial.println("step test: stop the wheels first (key 4) and wait for them to stand still");
    return;
  }
  control_record_start(periods(STEP_TEST_DURATION_S));
  set_both(TEST_DUTY, TEST_DUTY);
  running = Test::STEP;
  Serial.printf("step test: duty %.2f for %.1f s...\n", TEST_DUTY, STEP_TEST_DURATION_S);
}

void start_stability_test() {
  if (!wheels_driven()) {
    Serial.println("stability test: start the wheels first (e.g. key 1) and wait 2 s");
    return;
  }
  control_record_start(periods(STABILITY_TEST_DURATION_S));
  running = Test::STABILITY;
  Serial.printf("stability test: measuring for %.1f s...\n", STABILITY_TEST_DURATION_S);
}

// Steady speed, gain (rad/s per unit duty) and time to 63% of the steady speed.
void finish_step_test() {
  set_both(0.0f, 0.0f);
  const RecordedSample *s = control_record_samples();
  const int n = periods(STEP_TEST_DURATION_S);
  const int settled = periods(STEP_TEST_SETTLED_S);
  Serial.printf("step test result (duty %.2f, time resolution %.2f s):\n", TEST_DUTY, CONTROL_PERIOD_S);
  for (int w = 0; w < WHEEL_COUNT; w++) {
    float steady = 0.0f;
    for (int i = n - settled; i < n; i++) steady += s[i].velocity_raw[w];
    steady /= settled;
    int i63 = 0;
    while (i63 < n && s[i63].velocity_raw[w] < 0.632f * steady) i63++;
    Serial.printf("  %s  steady %+6.2f rad/s  gain K %5.1f rad/s per duty  tau %.2f s\n",
                  WHEEL_NAMES[w], steady, steady / TEST_DUTY, (i63 + 1) * CONTROL_PERIOD_S);
  }
}

void finish_stability_test() {
  const RecordedSample *s = control_record_samples();
  const int n = periods(STABILITY_TEST_DURATION_S);
  Serial.printf("stability test result (%.1f s, %d samples):\n", STABILITY_TEST_DURATION_S, n);
  for (int w = 0; w < WHEEL_COUNT; w++) {
    float mean = 0.0f;
    for (int i = 0; i < n; i++) mean += s[i].velocity[w];
    mean /= n;
    float var = 0.0f;
    for (int i = 0; i < n; i++) var += sq(s[i].velocity[w] - mean);
    const float std_dev = sqrtf(var / n);
    const float tick_mean = control_record_tick_speed(static_cast<Wheel>(w));
    const float std_ratio = std_dev / fabsf(mean);
    const float mean_error = fabsf(mean - tick_mean) / fabsf(tick_mean);
    const bool pass = std_ratio <= STABILITY_MAX_STD_RATIO && mean_error <= STABILITY_MAX_MEAN_ERROR;
    Serial.printf("  %s  mean %+6.2f rad/s  std %.3f (%.1f%%)  from ticks %+6.2f (diff %.1f%%)  %s\n",
                  WHEEL_NAMES[w], mean, std_dev, 100.0f * std_ratio, tick_mean,
                  100.0f * mean_error, pass ? "PASS" : "FAIL");
  }
}

void handle_serial() {
  while (Serial.available() > 0) {
    const char key = Serial.read();
    if (key == '4') {
      set_both(0.0f, 0.0f);
      step_duty = 0.0f;
      if (running != Test::NONE) Serial.println("test aborted");
      control_record_stop();
      running = Test::NONE;
      continue;
    }
    if (running != Test::NONE) continue;  // only 4 works while a test runs
    switch (key) {
      case '1': set_both(TEST_DUTY, TEST_DUTY); break;
      case '2': set_both(-TEST_DUTY, -TEST_DUTY); break;
      case '3': set_both(TEST_DUTY, -TEST_DUTY); break;
      case '+':
        step_duty = min(step_duty + TEST_DUTY_STEP, 1.0f);
        set_both(step_duty, step_duty);
        break;
      case '-':
        step_duty = max(step_duty - TEST_DUTY_STEP, 0.0f);
        set_both(step_duty, step_duty);
        break;
      case '5': start_step_test(); break;
      case '6': start_stability_test(); break;
      case 'z': encoders_read(zero); break;
      default: break;
    }
  }
}

void check_test() {
  if (running == Test::NONE || !control_record_done()) return;
  if (running == Test::STEP) finish_step_test();
  else finish_stability_test();
  control_record_stop();
  running = Test::NONE;
}

void print_status() {
  EncoderSnapshot enc[WHEEL_COUNT];
  encoders_read(enc);
  WheelState st[WHEEL_COUNT];
  control_get_state(st);
  for (int w = 0; w < WHEEL_COUNT; w++) {
    const MotorStatus m = motors_status(static_cast<Wheel>(w));
    Serial.printf("%s%s %6ld ticks  w %+6.2f rad/s  duty %+.2f (req %+.2f)  glitch %lu fast %lu",
                  w > 0 ? " | " : "", WHEEL_NAMES[w], (long)(enc[w].ticks - zero[w].ticks),
                  st[w].velocity, m.applied, m.requested,
                  (unsigned long)(enc[w].glitches - zero[w].glitches),
                  (unsigned long)(enc[w].fast - zero[w].fast));
  }
  Serial.println();
}

}  // namespace

void setup() {
  motors_begin();  // first, so the motors are off as soon as possible
  Serial.begin(SERIAL_BAUD);
  encoders_begin();
  control_begin();
  Serial.println("wheel firmware, block 3: speed. Keys: 1 fwd, 2 back, 3 opposite, 4 stop, "
                 "+/- duty, 5 step test, 6 stability test, z zero counters");
}

void loop() {
  handle_serial();
  motors_update();
  check_test();
  const unsigned long now = millis();
  if (running == Test::NONE && now - last_print_ms >= DEBUG_PERIOD_MS) {
    last_print_ms = now;
    print_status();
  }
}
