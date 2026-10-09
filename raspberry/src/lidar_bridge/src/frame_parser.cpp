#include "lidar_bridge/frame_parser.hpp"

#include <algorithm>
#include <utility>

namespace lidar_bridge
{

namespace
{
constexpr uint8_t SYNC0 = 0xA5;
constexpr uint8_t SYNC1 = 0x5A;

uint16_t u16(const uint8_t * p) {return p[0] | (uint16_t(p[1]) << 8);}
uint32_t u32(const uint8_t * p) {return u16(p) | (uint32_t(u16(p + 2)) << 16);}
}  // namespace

uint16_t crc16Ccitt(const uint8_t * data, size_t len, uint16_t crc)
{
  for (size_t i = 0; i < len; i++) {
    crc ^= uint16_t(data[i]) << 8;
    for (int b = 0; b < 8; b++) {
      crc = (crc & 0x8000) ? uint16_t((crc << 1) ^ 0x1021) : uint16_t(crc << 1);
    }
  }
  return crc;
}

FrameParser::FrameParser(Callback on_frame)
: on_frame_(std::move(on_frame)) {}

void FrameParser::feed(const uint8_t * data, size_t len)
{
  buf_.insert(buf_.end(), data, data + len);
  while (tryParse()) {
  }
}

bool FrameParser::tryParse()
{
  // Drop everything before the first sync pair
  const uint8_t sync[] = {SYNC0, SYNC1};
  auto start = std::search(buf_.begin(), buf_.end(), sync, sync + 2);
  if (start == buf_.end()) {
    // Keep a trailing A5: it may be the first half of the next sync
    const bool keep_last = !buf_.empty() && buf_.back() == SYNC0;
    buf_.erase(buf_.begin(), buf_.end() - (keep_last ? 1 : 0));
    return false;
  }
  buf_.erase(buf_.begin(), start);

  if (buf_.size() < HEADER_SIZE) {
    return false;
  }
  const uint8_t * p = buf_.data();
  const uint8_t version = p[2];
  const uint16_t bins = u16(p + 9);
  if (version != FRAME_VERSION || bins == 0 || bins > MAX_BINS) {
    buf_.erase(buf_.begin());  // false sync, look for the next one
    return true;
  }

  const size_t frame_len = HEADER_SIZE + 2 * size_t(bins) + 2;
  if (buf_.size() < frame_len) {
    return false;
  }
  if (crc16Ccitt(p + 2, frame_len - 4) != u16(p + frame_len - 2)) {
    crc_errors_++;
    buf_.erase(buf_.begin());
    return true;
  }

  ScanFrame frame;
  frame.seq = u16(p + 3);
  frame.scan_duration_us = u32(p + 5);
  frame.distances_mm.resize(bins);
  for (uint16_t i = 0; i < bins; i++) {
    frame.distances_mm[i] = u16(p + HEADER_SIZE + 2 * i);
  }
  buf_.erase(buf_.begin(), buf_.begin() + frame_len);
  frames_ok_++;
  on_frame_(frame);
  return true;
}

}  // namespace lidar_bridge
