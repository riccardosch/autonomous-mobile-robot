// Basic types shared by the exploration modules, without ROS.
#pragma once

#include <cstdint>
#include <vector>

namespace agv_explore {

struct Point { double x, y; };

struct Robot { double x, y, yaw; };

// An occupancy grid as published on /map or a Nav2 costmap (OccupancyGrid), not rotated.
struct Grid {
  int width = 0, height = 0;
  double resolution = 0.05;           // m per cell
  double origin_x = 0, origin_y = 0;  // world position of cell (0, 0)'s corner
  std::vector<int8_t> data;           // row-major from origin: -1 unknown, 0..100

  bool free(int i) const { return data[i] >= 0 && data[i] < 50; }
  bool unknown(int i) const { return data[i] < 0; }
  double x(int i) const { return origin_x + (i % width + 0.5) * resolution; }
  double y(int i) const { return origin_y + (i / width + 0.5) * resolution; }
  // Cell index of the world point (x, y), or -1 outside the grid.
  int index(double wx, double wy) const;
};

double distance(const Point &a, const Point &b);

}  // namespace agv_explore
