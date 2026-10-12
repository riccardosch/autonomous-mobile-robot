#include "motors.h"

#include <Arduino.h>

#include "config.h"

namespace {

struct MotorPins {
  int enable;
  int in_a;
  int in_b;
  uint8_t channel;
  bool inverted;
};

const MotorPins PINS[WHEEL_COUNT] = {
    {PIN_ENA, PIN_IN1, PIN_IN2, MOTOR_PWM_CHANNEL_LEFT, MOTOR_LEFT_INVERTED},
    {PIN_ENB, PIN_IN3, PIN_IN4, MOTOR_PWM_CHANNEL_RIGHT, MOTOR_RIGHT_INVERTED},
};

constexpr uint32_t PWM_MAX = (1u << MOTOR_PWM_RESOLUTION_BITS) - 1;
constexpr uint32_t REVERSE_WAIT_US = static_cast<uint32_t>(REVERSE_WAIT_S * 1e6f);

float requested[WHEEL_COUNT] = {};
float applied[WHEEL_COUNT] = {};
int8_t direction[WHEEL_COUNT] = {1, 1};  // direction the bridge drives, +1 forward

void write_bridge(int w, float magnitude) {
  const MotorPins &p = PINS[w];
  if (magnitude <= 0.0f) {
    digitalWrite(p.in_a, LOW);
    digitalWrite(p.in_b, LOW);
    ledcWrite(p.channel, MOTOR_BRAKE_ON_STOP ? PWM_MAX : 0);
    return;
  }
  const bool a_high = (direction[w] > 0) != p.inverted;
  digitalWrite(p.in_a, a_high ? HIGH : LOW);
  digitalWrite(p.in_b, a_high ? LOW : HIGH);
  ledcWrite(p.channel, lroundf(magnitude * PWM_MAX));
}

bool wheel_stopped(int w) {
  EncoderSnapshot enc[WHEEL_COUNT];
  encoders_read(enc);
  return micros() - enc[w].last_edge_us >= REVERSE_WAIT_US;
}

void apply(int w) {
  const float req = requested[w];
  const int8_t wanted = req > 0.0f ? 1 : (req < 0.0f ? -1 : direction[w]);
  if (wanted != direction[w]) {
    if (!wheel_stopped(w)) {
      // Still turning: let it coast to a stop with the old direction.
      applied[w] = 0.0f;
      write_bridge(w, 0.0f);
      return;
    }
    direction[w] = wanted;
    encoders_set_direction(static_cast<Wheel>(w), wanted);
  }
  applied[w] = req;
  write_bridge(w, fabsf(req));
}

}  // namespace

void motors_begin() {
  for (int w = 0; w < WHEEL_COUNT; w++) {
    const MotorPins &p = PINS[w];
    pinMode(p.in_a, OUTPUT);
    pinMode(p.in_b, OUTPUT);
    digitalWrite(p.in_a, LOW);
    digitalWrite(p.in_b, LOW);
    ledcSetup(p.channel, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_RESOLUTION_BITS);
    ledcAttachPin(p.enable, p.channel);
    ledcWrite(p.channel, 0);
  }
}

void motors_set(Wheel wheel, float duty) {
  if (isnan(duty)) duty = 0.0f;
  requested[wheel] = constrain(duty, -1.0f, 1.0f);
  apply(wheel);
}

MotorStatus motors_status(Wheel wheel) {
  return {requested[wheel], applied[wheel]};
}
