#pragma once

#include "encoders.h"

struct WheelState {
  float position;      // rad since start, + = forward
  float velocity;      // rad/s, filtered
  float velocity_raw;  // rad/s, before the filter
};

// One sample per control period, recorded for the tests.
struct RecordedSample {
  float velocity_raw[WHEEL_COUNT];
  float velocity[WHEEL_COUNT];
};

constexpr int RECORD_MAX_SAMPLES = 256;

// Starts the fixed-period control task (CONTROL_PERIOD_S, core CONTROL_TASK_CORE).
void control_begin();

// Consistent copy of the latest state of both wheels.
void control_get_state(WheelState out[WHEEL_COUNT]);

// Records the next `samples` periods (at most RECORD_MAX_SAMPLES).
void control_record_start(int samples);
void control_record_stop();
bool control_record_done();

// Valid once control_record_done() is true, until the next start.
const RecordedSample *control_record_samples();

// Mean speed over the recording from the tick counts alone, rad/s.
float control_record_tick_speed(Wheel wheel);
