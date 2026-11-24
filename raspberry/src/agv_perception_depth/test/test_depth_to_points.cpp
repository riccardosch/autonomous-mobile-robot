// Camera 0.105 m above the floor looking forward (optical z = robot x), as on the robot.
#include <gtest/gtest.h>

#include <vector>

#include "agv_perception_depth/depth_to_points.hpp"

using namespace agv_perception_depth;

namespace {
// Optical (x right, y down, z forward) -> robot (x forward, y left, z up).
const Pose kCamera{{0, 0, 1, -1, 0, 0, 0, -1, 0}, {0.04, -0.02, 0.105}};
const Intrinsics kK{100.0, 100.0, 2.0, 2.0};  // 5x5 image, centre pixel (2, 2)

std::vector<uint16_t> image_with(int u, int v, uint16_t mm) {
  std::vector<uint16_t> img(25, 0);
  img[v * 5 + u] = mm;
  return img;
}
Params one_pixel() { Params p; p.step = 1; return p; }
}  // namespace

TEST(DepthToPoints, CentrePixelIsStraightAhead) {
  const auto img = image_with(2, 2, 1000);   // 1 m ahead, at camera height
  const auto r = depth_to_points(img.data(), 5, 5, kK, kCamera, one_pixel());
  ASSERT_EQ(r.obstacles.size(), 1u);
  EXPECT_NEAR(r.obstacles[0].x, 1.04, 1e-6);
  EXPECT_NEAR(r.obstacles[0].y, -0.02, 1e-6);
  EXPECT_NEAR(r.obstacles[0].z, 0.105, 1e-6);
  EXPECT_TRUE(r.floor.empty());
}

TEST(DepthToPoints, LowRowHitsTheFloor) {
  // Pixel row v = 4 looks 2 px below the centre: z = 0.105 - (4 - 2) * d / 100 = 0 at d = 5.25 m.
  Params p = one_pixel();
  p.max_range = 6.0;
  const auto img = image_with(2, 4, 5250);
  const auto r = depth_to_points(img.data(), 5, 5, kK, kCamera, p);
  EXPECT_TRUE(r.obstacles.empty());
  ASSERT_EQ(r.floor.size(), 1u);
  EXPECT_NEAR(r.floor[0].z, 0.0, 1e-6);
}

TEST(DepthToPoints, RangeAndNoReadingAreSkipped) {
  for (uint16_t mm : {0, 400, 3000}) {       // no reading, too close, too far (defaults 0.5-1.5 m)
    const auto img = image_with(2, 2, mm);
    const auto r = depth_to_points(img.data(), 5, 5, kK, kCamera, one_pixel());
    EXPECT_TRUE(r.obstacles.empty() && r.floor.empty()) << mm;
  }
}

TEST(DepthToPoints, StepSkipsPixels) {
  const auto img = image_with(1, 1, 1000);   // not on the step-2 grid (0, 2, 4)
  Params p;
  p.step = 2;
  EXPECT_TRUE(depth_to_points(img.data(), 5, 5, kK, kCamera, p).obstacles.empty());
}
