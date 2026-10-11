// Wheel firmware for the ESP32 DevKit.
//
// Block 1: counts the transitions of the two wheel encoders and prints them on
// the serial monitor. Send 'z' to reset the counters.

#include <Arduino.h>

#include "config.h"
#include "encoders.h"

namespace {

unsigned long last_print_ms = 0;

void handle_serial() {
  while (Serial.available() > 0) {
    if (Serial.read() == 'z') {
      encoders_reset();
      Serial.println("counters reset");
    }
  }
}

void print_counts() {
  EncoderSnapshot enc[WHEEL_COUNT];
  encoders_read(enc);
  const EncoderSnapshot &l = enc[WHEEL_LEFT];
  const EncoderSnapshot &r = enc[WHEEL_RIGHT];
  Serial.printf("L %6ld ticks %7.3f rev glitch %lu fast %lu | R %6ld ticks %7.3f rev glitch %lu fast %lu\n",
                (long)l.ticks, l.ticks / ENCODER_TICKS_PER_REV, (unsigned long)l.glitches,
                (unsigned long)l.fast, (long)r.ticks, r.ticks / ENCODER_TICKS_PER_REV,
                (unsigned long)r.glitches, (unsigned long)r.fast);
}

}  // namespace

void setup() {
  Serial.begin(SERIAL_BAUD);
  encoders_begin();
  Serial.println("wheel firmware, block 1: encoders. Send 'z' to reset the counters.");
}

void loop() {
  handle_serial();
  const unsigned long now = millis();
  if (now - last_print_ms >= DEBUG_PERIOD_MS) {
    last_print_ms = now;
    print_counts();
  }
}
