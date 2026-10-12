#include "velocity.h"

#include <Arduino.h>

#include "config.h"

// The speed is the number of ticks between the last edge of the previous
// period and the last edge of this one, divided by the time between those two
// edges (M/T method). The resolution then depends on the edge timestamps
// (1 us), not on how many ticks fall in one period (~5 at 3 rad/s).

namespace {

constexpr float RAD_PER_TICK = 2.0f * PI / ENCODER_TICKS_PER_REV;
constexpr float FILTER_ALPHA = CONTROL_PERIOD_S / (VELOCITY_FILTER_TAU_S + CONTROL_PERIOD_S);
constexpr uint32_t ZERO_TIMEOUT_US = static_cast<uint32_t>(VELOCITY_ZERO_TIMEOUT_S * 1e6f);
constexpr uint32_t PERIOD_US = static_cast<uint32_t>(CONTROL_PERIOD_S * 1e6f);

struct Estimator {
  bool started;
  int32_t ticks;     // count at the last edge used
  uint32_t edge_us;  // time of that edge
  VelocityEstimate v;
};

Estimator est[WHEEL_COUNT] = {};

void update(Estimator &e, const EncoderSnapshot &enc, uint32_t now_us) {
  if (!e.started) {
    e = {true, enc.ticks, enc.last_edge_us, {0.0f, 0.0f}};
    return;
  }
  const int32_t dticks = enc.ticks - e.ticks;
  if (dticks != 0) {
    uint32_t dt_us = enc.last_edge_us - e.edge_us;
    // Starting from standstill the previous edge is old: use one period.
    if (dt_us > ZERO_TIMEOUT_US) dt_us = PERIOD_US;
    e.v.raw = dticks * RAD_PER_TICK / (dt_us * 1e-6f);
    e.ticks = enc.ticks;
    e.edge_us = enc.last_edge_us;
  } else {
    const uint32_t since_edge_us = now_us - e.edge_us;
    if (since_edge_us > ZERO_TIMEOUT_US) {
      e.v.raw = 0.0f;
    } else {
      // No edge in this period: the wheel is turning at most one tick in the
      // time since the last edge, so the speed can only go down.
      const float bound = RAD_PER_TICK / (since_edge_us * 1e-6f);
      if (fabsf(e.v.raw) > bound) e.v.raw = copysignf(bound, e.v.raw);
    }
  }
  e.v.filtered += FILTER_ALPHA * (e.v.raw - e.v.filtered);
}

}  // namespace

void velocity_update(const EncoderSnapshot enc[WHEEL_COUNT], uint32_t now_us,
                     VelocityEstimate out[WHEEL_COUNT]) {
  for (int w = 0; w < WHEEL_COUNT; w++) {
    update(est[w], enc[w], now_us);
    out[w] = est[w].v;
  }
}
