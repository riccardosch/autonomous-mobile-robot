#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

namespace lidar_bridge
{

// One revolution as sent by the ESP32 firmware (firmware_lidar, esp32c3 env).
struct ScanFrame
{
  uint16_t seq = 0;
  uint32_t scan_duration_us = 0;
  std::vector<uint16_t> distances_mm;  // index = angle in bins, 0 = no echo
};

// Decoder for the binary frame, little-endian:
//   sync 2 (A5 5A) | version 1 | seq 2 | scan_duration_us 4 | bins 2 |
//   distances_mm 2*bins | crc16 2 (CCITT-FALSE over everything after sync)
//
// Bytes can arrive in chunks of any size; every complete frame with a valid CRC
// is passed to the callback.
class FrameParser
{
public:
  using Callback = std::function<void (const ScanFrame &)>;

  static constexpr uint8_t FRAME_VERSION = 1;
  static constexpr uint16_t MAX_BINS = 3600;

  explicit FrameParser(Callback on_frame);

  void feed(const uint8_t * data, size_t len);

  uint32_t framesOk() const {return frames_ok_;}
  uint32_t crcErrors() const {return crc_errors_;}

private:
  static constexpr size_t HEADER_SIZE = 11;  // sync + version + seq + duration + bins

  bool tryParse();  // false when more bytes are needed

  Callback on_frame_;
  std::vector<uint8_t> buf_;
  uint32_t frames_ok_ = 0;
  uint32_t crc_errors_ = 0;
};

uint16_t crc16Ccitt(const uint8_t * data, size_t len, uint16_t crc = 0xFFFF);

}  // namespace lidar_bridge
