#pragma once

#include <stddef.h>
#include <stdint.h>

// Decoder for the YDLIDAR X2/X2L packet stream (also used by the YDS2MD-1).
//
// Packet layout, little-endian:
//   PH   2  header 0x55AA (AA 55 on the wire)
//   CT   1  bit 0 = 1: first packet of a new revolution;
//           in that packet bits 7:1 are the scan frequency in 0.1 Hz
//   LSN  1  number of samples
//   FSA  2  first sample angle, bit 0 = check bit, angle = (FSA >> 1) / 64 deg
//   LSA  2  last sample angle, same encoding
//   CS   2  XOR of the 16-bit words PH, FSA, every Si, CT|LSN<<8, LSA
//   Si   2*LSN  distance samples, distance_mm = Si / 4
//
// Feed bytes one at a time with feed(). When it returns true a complete packet
// with a valid checksum has been decoded and its points can be read.
class X2Parser {
 public:
  static constexpr uint8_t MAX_SAMPLES = 40;

  struct Point {
    float angle_deg;       // corrected angle, [0, 360)
    uint16_t distance_mm;  // 0 = no echo
  };

  bool feed(uint8_t byte);

  // Valid only after feed() returned true, until the next byte is fed.
  bool isRingStart() const { return (buf_[2] & 0x01) != 0; }
  float ringScanFreqHz() const { return (buf_[2] >> 1) * 0.1f; }
  uint8_t sampleCount() const { return buf_[3]; }
  Point point(uint8_t i) const;

  uint32_t packetsOk() const { return packets_ok_; }
  uint32_t checksumErrors() const { return checksum_errors_; }
  uint32_t framingErrors() const { return framing_errors_; }

 private:
  static constexpr size_t HEADER_SIZE = 10;
  static constexpr uint16_t FULL_TURN_Q6 = 360 * 64;

  uint16_t word(size_t offset) const {
    return buf_[offset] | (uint16_t(buf_[offset + 1]) << 8);
  }
  bool checksumValid() const;
  void decodeAngles();

  uint8_t buf_[HEADER_SIZE + 2 * MAX_SAMPLES] = {};
  size_t pos_ = 0;
  size_t expected_ = 0;

  // Angles in 1/64 degree units, set by decodeAngles()
  float first_angle_q6_ = 0;
  float step_q6_ = 0;
  float last_step_q6_ = 0;

  uint32_t packets_ok_ = 0;
  uint32_t checksum_errors_ = 0;
  uint32_t framing_errors_ = 0;
};
