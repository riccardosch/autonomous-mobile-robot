#include "control.h"

#include <Arduino.h>

#include "config.h"
#include "velocity.h"

namespace {

constexpr float RAD_PER_TICK = 2.0f * PI / ENCODER_TICKS_PER_REV;
constexpr uint32_t PERIOD_MS = static_cast<uint32_t>(CONTROL_PERIOD_S * 1000.0f + 0.5f);

static_assert(STABILITY_TEST_DURATION_S / CONTROL_PERIOD_S <= RECORD_MAX_SAMPLES,
              "stability test longer than the record buffer");
static_assert(STEP_TEST_DURATION_S / CONTROL_PERIOD_S <= RECORD_MAX_SAMPLES,
              "step test longer than the record buffer");

// Shared between the control task (core 0) and loop() (core 1).
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
WheelState state[WHEEL_COUNT] = {};
RecordedSample samples[RECORD_MAX_SAMPLES];
int record_target = 0;
int record_count = 0;
int32_t record_start_ticks[WHEEL_COUNT] = {};
int32_t record_end_ticks[WHEEL_COUNT] = {};

void control_task(void *) {
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(PERIOD_MS));
    EncoderSnapshot enc[WHEEL_COUNT];
    encoders_read(enc);
    VelocityEstimate v[WHEEL_COUNT];
    velocity_update(enc, micros(), v);

    portENTER_CRITICAL(&mux);
    for (int w = 0; w < WHEEL_COUNT; w++) {
      state[w] = {enc[w].ticks * RAD_PER_TICK, v[w].filtered, v[w].raw};
    }
    if (record_count < record_target) {
      RecordedSample &s = samples[record_count];
      for (int w = 0; w < WHEEL_COUNT; w++) {
        if (record_count == 0) record_start_ticks[w] = enc[w].ticks;
        record_end_ticks[w] = enc[w].ticks;
        s.velocity_raw[w] = v[w].raw;
        s.velocity[w] = v[w].filtered;
      }
      record_count++;
    }
    portEXIT_CRITICAL(&mux);
  }
}

}  // namespace

void control_begin() {
  xTaskCreatePinnedToCore(control_task, "control", CONTROL_TASK_STACK, nullptr,
                          CONTROL_TASK_PRIORITY, nullptr, CONTROL_TASK_CORE);
}

void control_get_state(WheelState out[WHEEL_COUNT]) {
  portENTER_CRITICAL(&mux);
  for (int w = 0; w < WHEEL_COUNT; w++) out[w] = state[w];
  portEXIT_CRITICAL(&mux);
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
