// Wheel firmware for the ESP32 DevKit.
//
// Block 2: open-loop motor control from the serial monitor, with the signed
// encoder counts printed every DEBUG_PERIOD_MS. Single keys, no Enter needed:
//   1  both wheels forward at TEST_DUTY
//   2  both wheels backward at TEST_DUTY
//   3  left forward, right backward
//   4  stop both motors
//   +  both forward, duty up by TEST_DUTY_STEP (to find the dead zone)
//   -  duty down by TEST_DUTY_STEP
//   z  reset the encoder counters

#include <Arduino.h>

#include "config.h"
#include "encoders.h"
#include "motors.h"

namespace {

unsigned long last_print_ms = 0;
float step_duty = 0.0f;  // duty set by the + and - keys

void set_both(float left, float right) {
  motors_set(WHEEL_LEFT, left);
  motors_set(WHEEL_RIGHT, right);
}

void handle_serial() {
  while (Serial.available() > 0) {
    switch (Serial.read()) {
      case '1': set_both(TEST_DUTY, TEST_DUTY); break;
      case '2': set_both(-TEST_DUTY, -TEST_DUTY); break;
      case '3': set_both(TEST_DUTY, -TEST_DUTY); break;
      case '4': set_both(0.0f, 0.0f); step_duty = 0.0f; break;
      case '+':
        step_duty = min(step_duty + TEST_DUTY_STEP, 1.0f);
        set_both(step_duty, step_duty);
        break;
      case '-':
        step_duty = max(step_duty - TEST_DUTY_STEP, 0.0f);
        set_both(step_duty, step_duty);
        break;
      case 'z':
        encoders_reset();
        Serial.println("counters reset");
        break;
      default: break;
    }
  }
}

void print_wheel(const char *name, const EncoderSnapshot &e, const MotorStatus &m) {
  Serial.printf("%s %6ld ticks %7.3f rev duty %+.2f (req %+.2f) glitch %lu fast %lu", name,
                (long)e.ticks, e.ticks / ENCODER_TICKS_PER_REV, m.applied, m.requested,
                (unsigned long)e.glitches, (unsigned long)e.fast);
}

void print_status() {
  EncoderSnapshot enc[WHEEL_COUNT];
  encoders_read(enc);
  print_wheel("L", enc[WHEEL_LEFT], motors_status(WHEEL_LEFT));
  Serial.print(" | ");
  print_wheel("R", enc[WHEEL_RIGHT], motors_status(WHEEL_RIGHT));
  Serial.println();
}

}  // namespace

void setup() {
  motors_begin();  // first, so the motors are off as soon as possible
  Serial.begin(SERIAL_BAUD);
  encoders_begin();
  Serial.println("wheel firmware, block 2: motors. Keys: 1 fwd, 2 back, 3 opposite, 4 stop, +/- duty, z reset");
}

void loop() {
  handle_serial();
  motors_update();
  const unsigned long now = millis();
  if (now - last_print_ms >= DEBUG_PERIOD_MS) {
    last_print_ms = now;
    print_status();
  }
}
