#include <gtest/gtest.h>

#include <algorithm>
#include <vector>

#include "lidar_bridge/frame_parser.hpp"

using lidar_bridge::FrameParser;
using lidar_bridge::ScanFrame;

namespace
{

// Builds a frame exactly like the firmware does (firmware_lidar/src/main.cpp).
std::vector<uint8_t> makeFrame(uint16_t seq, uint32_t duration_us, const std::vector<uint16_t> & d)
{
  std::vector<uint8_t> f = {0xA5, 0x5A, FrameParser::FRAME_VERSION,
    uint8_t(seq), uint8_t(seq >> 8),
    uint8_t(duration_us), uint8_t(duration_us >> 8), uint8_t(duration_us >> 16),
    uint8_t(duration_us >> 24),
    uint8_t(d.size()), uint8_t(d.size() >> 8)};
  for (uint16_t v : d) {
    f.push_back(uint8_t(v));
    f.push_back(uint8_t(v >> 8));
  }
  const uint16_t crc = lidar_bridge::crc16Ccitt(f.data() + 2, f.size() - 2);
  f.push_back(uint8_t(crc));
  f.push_back(uint8_t(crc >> 8));
  return f;
}

}  // namespace

TEST(Crc16, MatchesCcittFalseCheckValue)
{
  const uint8_t data[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  EXPECT_EQ(lidar_bridge::crc16Ccitt(data, sizeof(data)), 0x29B1);
}

TEST(FrameParser, DecodesFrameAfterGarbage)
{
  std::vector<ScanFrame> got;
  FrameParser parser([&](const ScanFrame & f) {got.push_back(f);});

  std::vector<uint8_t> stream = {0x00, 0xA5, 0x13, 0xA5};
  const auto frame = makeFrame(42, 125000, {0, 1000, 65535, 120});
  stream.insert(stream.end(), frame.begin(), frame.end());
  parser.feed(stream.data(), stream.size());

  ASSERT_EQ(got.size(), 1u);
  EXPECT_EQ(got[0].seq, 42);
  EXPECT_EQ(got[0].scan_duration_us, 125000u);
  EXPECT_EQ(got[0].distances_mm, (std::vector<uint16_t>{0, 1000, 65535, 120}));
}

TEST(FrameParser, HandlesFramesSplitAcrossReads)
{
  std::vector<ScanFrame> got;
  FrameParser parser([&](const ScanFrame & f) {got.push_back(f);});

  std::vector<uint8_t> stream;
  for (uint16_t seq = 1; seq <= 3; seq++) {
    const auto f = makeFrame(seq, 125000, std::vector<uint16_t>(360, seq * 100));
    stream.insert(stream.end(), f.begin(), f.end());
  }
  for (size_t i = 0; i < stream.size(); i += 7) {
    parser.feed(stream.data() + i, std::min<size_t>(7, stream.size() - i));
  }

  ASSERT_EQ(got.size(), 3u);
  EXPECT_EQ(got[2].seq, 3);
  EXPECT_EQ(got[2].distances_mm[359], 300);
}

TEST(FrameParser, RejectsBadCrcAndRecovers)
{
  std::vector<ScanFrame> got;
  FrameParser parser([&](const ScanFrame & f) {got.push_back(f);});

  auto bad = makeFrame(1, 125000, std::vector<uint16_t>(360, 500));
  bad[20] ^= 0xFF;
  const auto good = makeFrame(2, 125000, std::vector<uint16_t>(360, 600));
  bad.insert(bad.end(), good.begin(), good.end());
  parser.feed(bad.data(), bad.size());

  EXPECT_EQ(parser.crcErrors(), 1u);
  ASSERT_EQ(got.size(), 1u);
  EXPECT_EQ(got[0].seq, 2);
}
