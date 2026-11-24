// Depth image -> points on the robot, split into obstacles and floor. No ROS here:
// the node feeds it the image, the camera intrinsics and the camera pose.
#pragma once

#include <cstdint>
#include <vector>

namespace agv_perception_depth {

struct Point { float x, y, z; };

// Pinhole intrinsics from CameraInfo.K (pixels).
struct Intrinsics { double fx, fy, cx, cy; };

// Pose of the camera (optical frame) in the output frame: p_out = R * p_cam + t.
struct Pose { double R[9]; double t[3]; };

struct Params {
  int step = 4;               // use one pixel every `step` in both directions
  double min_range = 0.5;     // m, along the optical axis; Kinect v1 is blind closer
  double max_range = 1.5;     // m; depth noise and mount tilt grow with distance
  double min_height = 0.05;   // m above the floor: lower points are floor (or noise)
  double max_height = 0.15;   // m: higher points cannot hit the robot
  double floor_band = 0.03;   // m: |z| below this is floor, used to clear old obstacles
};

struct Result {
  std::vector<Point> obstacles;  // min_height <= z <= max_height
  std::vector<Point> floor;      // |z| <= floor_band
};

// depth_mm: row-major, width*height values in millimetres, 0 = no reading.
// The output frame must have z = 0 on the floor (base_footprint).
Result depth_to_points(const uint16_t *depth_mm, int width, int height,
                       const Intrinsics &k, const Pose &pose, const Params &p);

}  // namespace agv_perception_depth
