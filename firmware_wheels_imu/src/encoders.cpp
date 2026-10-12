#include "encoders.h"

#include <Arduino.h>

#include "config.h"

namespace {

struct EncoderState {
  volatile int32_t ticks;
  volatile uint32_t last_edge_us;
  volatile uint32_t glitches;
  volatile uint32_t fast;
  volatile int8_t direction;  // +1 or -1, added to ticks at every edge
};

EncoderState state[WHEEL_COUNT] = {};

// Protects the state shared between the ISRs and the readers. A spinlock,
// because readers may run on either core.
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR on_edge(EncoderState &s) {
  const uint32_t now = micros();
  portENTER_CRITICAL_ISR(&mux);
  const uint32_t interval = now - s.last_edge_us;
  if (interval < ENCODER_GLITCH_US) {
    s.glitches++;
  } else {
    if (interval < ENCODER_FAST_US) s.fast++;
    s.ticks += s.direction;
    s.last_edge_us = now;
  }
  portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR isr_left() { on_edge(state[WHEEL_LEFT]); }
void IRAM_ATTR isr_right() { on_edge(state[WHEEL_RIGHT]); }

}  // namespace

void encoders_begin() {
  for (int i = 0; i < WHEEL_COUNT; i++) state[i].direction = 1;
  pinMode(PIN_ENC_LEFT, INPUT_PULLUP);
  pinMode(PIN_ENC_RIGHT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_LEFT), isr_left, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_RIGHT), isr_right, CHANGE);
}

void encoders_read(EncoderSnapshot out[WHEEL_COUNT]) {
  portENTER_CRITICAL(&mux);
  for (int i = 0; i < WHEEL_COUNT; i++) {
    out[i].ticks = state[i].ticks;
    out[i].last_edge_us = state[i].last_edge_us;
    out[i].glitches = state[i].glitches;
    out[i].fast = state[i].fast;
  }
  portEXIT_CRITICAL(&mux);
}

void encoders_set_direction(Wheel wheel, int8_t direction) {
  portENTER_CRITICAL(&mux);
  state[wheel].direction = direction;
  portEXIT_CRITICAL(&mux);
}
