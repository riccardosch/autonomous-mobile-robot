#include "agv_perception_depth/depth_to_points.hpp"

#include <cmath>

namespace agv_perception_depth {

Result depth_to_points(const uint16_t *depth_mm, int width, int height,
                       const Intrinsics &k, const Pose &pose, const Params &p) {
  Result out;
  const int step = p.step < 1 ? 1 : p.step;
  const double *R = pose.R;
  for (int v = 0; v < height; v += step) {
    for (int u = 0; u < width; u += step) {
      const uint16_t mm = depth_mm[v * width + u];
      if (mm == 0) continue;                       // no reading
      const double zc = mm * 0.001;
      if (zc < p.min_range || zc > p.max_range) continue;
      // Pixel -> point in the optical frame (z forward, x right, y down).
      const double xc = (u - k.cx) * zc / k.fx;
      const double yc = (v - k.cy) * zc / k.fy;
      // Optical frame -> output frame.
      const Point q{
        static_cast<float>(R[0] * xc + R[1] * yc + R[2] * zc + pose.t[0]),
        static_cast<float>(R[3] * xc + R[4] * yc + R[5] * zc + pose.t[1]),
        static_cast<float>(R[6] * xc + R[7] * yc + R[8] * zc + pose.t[2])};
      if (q.z >= p.min_height && q.z <= p.max_height) {
        out.obstacles.push_back(q);
      } else if (std::fabs(q.z) <= p.floor_band) {
        out.floor.push_back(q);
      }
    }
  }
  return out;
}

}  // namespace agv_perception_depth
