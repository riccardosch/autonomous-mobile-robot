// Wheel firmware for the ESP32 DevKit.
//
// Block 5: closed-loop wheel speed (PI + feedforward) with a command
// watchdog, commanded from the serial monitor or from tools/cmd_sender.py.
// Lines "v <left> <right>" (rad/s, ended by a newline) set the targets; they
// are meant for scripts. Single keys, no Enter needed:
//   1  both wheels forward at TEST_SPEED
//   2  both wheels backward at TEST_SPEED
//   3  left forward, right backward
//   4  stop both motors (also aborts a running test)
//   +  both forward, target up by TEST_SPEED_STEP (up to SPEED_MAX)
//   -  target down by TEST_SPEED_STEP
//   5  open-loop step test: TEST_DUTY from standstill, prints steady speed, gain and time constant
//   6  stability test: with the wheels turning, prints mean and spread of the speed
//      and, in closed loop, the error from the target
//   7  closed-loop step test: TEST_SPEED from standstill, prints rise time, overshoot, error
//   p/o  KP up/down by TUNE_FACTOR      i/u  KI up/down by TUNE_FACTOR
//   w  watchdog on/off (on at boot): with it on, keys 1-3 and +/- drive the
//      wheels only for CMD_TIMEOUT_S; tests 6 and 7 need it off
//   z  zero the displayed tick and glitch counters

#include <Arduino.h>

#include "config.h"
#include "control.h"
#include "encoders.h"
#include "motors.h"

namespace {

enum class Test { NONE, STEP, STABILITY, CL_STEP };

const char *const WHEEL_NAMES[WHEEL_COUNT] = {"L", "R"};

unsigned long last_print_ms = 0;
float step_target = 0.0f;  // target set by the + and - keys
Test running = Test::NONE;
bool was_tripped = false;
char line[32];      // "v <left> <right>" being received
int line_len = -1;  // -1: not receiving a line
EncoderSnapshot zero[WHEEL_COUNT] = {};  // counters at the last 'z'

int periods(float seconds) { return lroundf(seconds / CONTROL_PERIOD_S); }

float speed_tolerance(float target) {
  return max(SPEED_MAX_ERROR_RATIO * fabsf(target), SPEED_MAX_ERROR_ABS);
}

bool wheels_stopped() {
  WheelState st[WHEEL_COUNT];
  control_get_state(st);
  for (int w = 0; w < WHEEL_COUNT; w++) {
    if (st[w].velocity_raw != 0.0f || st[w].duty != 0.0f) return false;
  }
  return true;
}

bool wheels_driven() {
  WheelState st[WHEEL_COUNT];
  control_get_state(st);
  for (int w = 0; w < WHEEL_COUNT; w++) {
    if (st[w].duty == 0.0f) return false;
  }
  return true;
}

bool watchdog_off(const char *name) {
  if (control_watchdog_on()) Serial.printf("%s: turn the watchdog off first (key w)\n", name);
  return !control_watchdog_on();
}

bool start_from_standstill(Test test, float seconds, const char *name) {
  if (!wheels_stopped()) {
    Serial.printf("%s: stop the wheels first (key 4) and wait for them to stand still\n", name);
    return false;
  }
  control_record_start(periods(seconds));
  running = test;
  Serial.printf("%s: running for %.1f s...\n", name, seconds);
  return true;
}

void start_stability_test() {
  if (!watchdog_off("stability test")) return;
  if (!wheels_driven()) {
    Serial.println("stability test: start the wheels first (e.g. key 1) and wait 2 s");
    return;
  }
  control_record_start(periods(STABILITY_TEST_DURATION_S));
  running = Test::STABILITY;
  Serial.printf("stability test: measuring for %.1f s...\n", STABILITY_TEST_DURATION_S);
}

float mean_of(const RecordedSample *s, int from, int to, int w) {
  float sum = 0.0f;
  for (int i = from; i < to; i++) sum += s[i].velocity[w];
  return sum / (to - from);
}

// Steady speed, gain (rad/s per unit duty) and time to 63% of the steady speed.
void finish_step_test() {
  control_set_duty(0.0f, 0.0f);
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

// 10-90% rise time, overshoot and steady error of the filtered speed.
void finish_cl_step_test() {
  control_set_target(0.0f, 0.0f);
  const RecordedSample *s = control_record_samples();
  const int n = periods(CL_STEP_TEST_DURATION_S);
  const int settled = periods(STEP_TEST_SETTLED_S);
  Serial.printf("closed-loop step result (target %.1f rad/s, time resolution %.2f s):\n", TEST_SPEED,
                CONTROL_PERIOD_S);
  for (int w = 0; w < WHEEL_COUNT; w++) {
    int i10 = 0;
    while (i10 < n && s[i10].velocity[w] < 0.1f * TEST_SPEED) i10++;
    int i90 = i10;
    while (i90 < n && s[i90].velocity[w] < 0.9f * TEST_SPEED) i90++;
    float peak = 0.0f;
    for (int i = 0; i < n; i++) peak = max(peak, s[i].velocity[w]);
    const float steady = mean_of(s, n - settled, n, w);
    const float rise = (i90 - i10) * CONTROL_PERIOD_S;
    const float overshoot = max(peak - TEST_SPEED, 0.0f) / TEST_SPEED;
    const float error = steady - TEST_SPEED;
    const bool pass = i90 < n && rise <= CL_STEP_MAX_RISE_S && overshoot <= CL_STEP_MAX_OVERSHOOT &&
                      fabsf(error) <= speed_tolerance(TEST_SPEED);
    Serial.printf("  %s  rise %.2f s  overshoot %.1f%%  steady %+6.2f rad/s (error %+.2f)  %s\n",
                  WHEEL_NAMES[w], rise, 100.0f * overshoot, steady, error, pass ? "PASS" : "FAIL");
  }
}

void finish_stability_test() {
  WheelState st[WHEEL_COUNT];
  control_get_state(st);
  const RecordedSample *s = control_record_samples();
  const int n = periods(STABILITY_TEST_DURATION_S);
  Serial.printf("stability test result (%.1f s, %d samples):\n", STABILITY_TEST_DURATION_S, n);
  for (int w = 0; w < WHEEL_COUNT; w++) {
    const float mean = mean_of(s, 0, n, w);
    float var = 0.0f;
    for (int i = 0; i < n; i++) var += sq(s[i].velocity[w] - mean);
    const float std_dev = sqrtf(var / n);
    const float tick_mean = control_record_tick_speed(static_cast<Wheel>(w));
    const float std_ratio = std_dev / fabsf(mean);
    const float mean_error = fabsf(mean - tick_mean) / fabsf(tick_mean);
    bool pass = std_ratio <= STABILITY_MAX_STD_RATIO && mean_error <= STABILITY_MAX_MEAN_ERROR;
    Serial.printf("  %s  mean %+6.2f rad/s  std %.3f (%.1f%%)  from ticks %+6.2f (diff %.1f%%)",
                  WHEEL_NAMES[w], mean, std_dev, 100.0f * std_ratio, tick_mean, 100.0f * mean_error);
    if (st[w].target != 0.0f) {
      const float error = mean - st[w].target;
      pass = pass && fabsf(error) <= speed_tolerance(st[w].target);
      Serial.printf("  target %+6.2f (error %+.2f)", st[w].target, error);
    }
    Serial.printf("  %s\n", pass ? "PASS" : "FAIL");
  }
}

// Run-time tuning of the controller gains (they reset to PID_KP and PID_KI at boot).
void tune(char key) {
  float kp, ki;
  control_get_gains(kp, ki);
  if (key == 'p') kp *= TUNE_FACTOR;
  if (key == 'o') kp /= TUNE_FACTOR;
  if (key == 'i') ki *= TUNE_FACTOR;
  if (key == 'u') ki /= TUNE_FACTOR;
  control_set_gains(kp, ki);
  Serial.printf("gains: KP %.4f  KI %.4f\n", kp, ki);
}

// A "v <left> <right>" line from a script.
void handle_line_char(char c) {
  if (c != '\n' && c != '\r') {
    if (line_len < static_cast<int>(sizeof(line)) - 1) line[line_len++] = c;
    return;
  }
  line[line_len] = '\0';
  line_len = -1;
  float left, right;
  if (running == Test::NONE && sscanf(line, "%f %f", &left, &right) == 2) {
    control_set_target(left, right);
  }
}

void handle_serial() {
  while (Serial.available() > 0) {
    const char key = Serial.read();
    if (line_len >= 0) {
      handle_line_char(key);
      continue;
    }
    if (key == 'v') {
      line_len = 0;
      continue;
    }
    if (key == '4') {
      control_set_target(0.0f, 0.0f);
      step_target = 0.0f;
      if (running != Test::NONE) Serial.println("test aborted");
      control_record_stop();
      running = Test::NONE;
      continue;
    }
    if (running != Test::NONE) continue;  // only 4 works while a test runs
    switch (key) {
      case '1': step_target = TEST_SPEED; control_set_target(TEST_SPEED, TEST_SPEED); break;
      case '2': step_target = 0.0f; control_set_target(-TEST_SPEED, -TEST_SPEED); break;
      case '3': step_target = 0.0f; control_set_target(TEST_SPEED, -TEST_SPEED); break;
      case '+':
        step_target = min(step_target + TEST_SPEED_STEP, SPEED_MAX);
        control_set_target(step_target, step_target);
        break;
      case '-':
        step_target = max(step_target - TEST_SPEED_STEP, 0.0f);
        control_set_target(step_target, step_target);
        break;
      case '5':
        if (start_from_standstill(Test::STEP, STEP_TEST_DURATION_S, "open-loop step test")) {
          control_set_duty(TEST_DUTY, TEST_DUTY);
        }
        break;
      case '6': start_stability_test(); break;
      case '7':
        if (watchdog_off("closed-loop step test") && start_from_standstill(Test::CL_STEP, CL_STEP_TEST_DURATION_S, "closed-loop step test")) {
          control_set_target(TEST_SPEED, TEST_SPEED);
        }
        break;
      case 'p': case 'o': case 'i': case 'u': tune(key); break;
      case 'w':
        control_set_watchdog(!control_watchdog_on());
        Serial.printf("watchdog %s\n", control_watchdog_on() ? "on" : "off");
        break;
      case 'z': encoders_read(zero); break;
      default: break;
    }
  }
}

void check_watchdog() {
  const bool tripped = control_watchdog_tripped();
  if (tripped && !was_tripped) {
    Serial.printf("watchdog: no target for %.2f s, motors off\n", CMD_TIMEOUT_S);
    step_target = 0.0f;
  }
  was_tripped = tripped;
}

void check_test() {
  if (running == Test::NONE || !control_record_done()) return;
  switch (running) {
    case Test::STEP: finish_step_test(); break;
    case Test::CL_STEP: finish_cl_step_test(); break;
    default: finish_stability_test(); break;
  }
  control_record_stop();
  running = Test::NONE;
}

void print_status() {
  EncoderSnapshot enc[WHEEL_COUNT];
  encoders_read(enc);
  WheelState st[WHEEL_COUNT];
  control_get_state(st);
  for (int w = 0; w < WHEEL_COUNT; w++) {
    Serial.printf("%s%s %6ld ticks  w %+6.2f rad/s  target %+5.2f  duty %+.2f  glitch %lu fast %lu",
                  w > 0 ? " | " : "", WHEEL_NAMES[w], (long)(enc[w].ticks - zero[w].ticks),
                  st[w].velocity, st[w].target, st[w].duty,
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
  Serial.println("wheel firmware, block 5: speed PID + watchdog. Keys: 1 fwd, 2 back, 3 opposite, "
                 "4 stop, +/- target, 5 open-loop step, 6 stability, 7 closed-loop step, p/o KP, "
                 "i/u KI, w watchdog on/off, z zero counters");
  Serial.printf("watchdog %s\n", control_watchdog_on() ? "on" : "off");
}

void loop() {
  handle_serial();
  check_watchdog();
  check_test();
  const unsigned long now = millis();
  if (running == Test::NONE && now - last_print_ms >= DEBUG_PERIOD_MS) {
    last_print_ms = now;
    print_status();
  }
}
