#include "x2_parser.h"

#include <math.h>

namespace {
constexpr uint8_t PH_LOW = 0xAA;
constexpr uint8_t PH_HIGH = 0x55;
constexpr uint16_t PH = 0x55AA;
// A packet never spans more than this: a larger gap between FSA and LSA means
// a corrupted angle, so the step of the previous packet is reused instead.
constexpr float MAX_PACKET_SPAN_Q6 = 30 * 64;
}  // namespace

bool X2Parser::feed(uint8_t byte) {
  if (pos_ == 0) {
    if (byte == PH_LOW) buf_[pos_++] = byte;
    return false;
  }
  if (pos_ == 1) {
    if (byte == PH_HIGH) {
      buf_[pos_++] = byte;
    } else if (byte != PH_LOW) {
      pos_ = 0;  // a repeated AA may still be the start of a header
    }
    return false;
  }

  buf_[pos_++] = byte;

  if (pos_ == HEADER_SIZE) {
    const uint8_t lsn = buf_[3];
    const bool angles_ok = (buf_[4] & 0x01) && (buf_[6] & 0x01);  // check bits
    if (lsn == 0 || lsn > MAX_SAMPLES || !angles_ok) {
      framing_errors_++;
      pos_ = 0;
      return false;
    }
    expected_ = HEADER_SIZE + 2 * lsn;
    return false;
  }

  if (pos_ < HEADER_SIZE || pos_ < expected_) return false;

  pos_ = 0;
  if (!checksumValid()) {
    checksum_errors_++;
    return false;
  }
  decodeAngles();
  packets_ok_++;
  return true;
}

bool X2Parser::checksumValid() const {
  const uint8_t lsn = buf_[3];
  uint16_t cs = PH ^ word(4) ^ (buf_[2] | (uint16_t(lsn) << 8)) ^ word(6);
  for (uint8_t i = 0; i < lsn; i++) cs ^= word(HEADER_SIZE + 2 * i);
  return cs == word(8);
}

void X2Parser::decodeAngles() {
  const uint8_t lsn = buf_[3];
  const float first = word(4) >> 1;
  const float last = word(6) >> 1;
  first_angle_q6_ = first;

  if (lsn == 1) {
    step_q6_ = 0;
    return;
  }
  float span = last - first;
  if (span < 0) span += FULL_TURN_Q6;  // packet crosses 0 degrees
  if (span > MAX_PACKET_SPAN_Q6) {
    step_q6_ = last_step_q6_;
    return;
  }
  step_q6_ = span / (lsn - 1);
  last_step_q6_ = step_q6_;
}

X2Parser::Point X2Parser::point(uint8_t i) const {
  const uint16_t raw = word(HEADER_SIZE + 2 * i);
  const float distance = raw / 4.0f;

  float angle = (first_angle_q6_ + step_q6_ * i) / 64.0f;
  if (distance > 0) {
    // Correction for the offset between laser and sensor, from the X2 datasheet
    angle += atanf(21.8f * (155.3f - distance) / (155.3f * distance)) * (180.0f / M_PI);
  }
  angle = fmodf(angle, 360.0f);
  if (angle < 0) angle += 360.0f;

  return {angle, uint16_t(raw / 4)};
}
