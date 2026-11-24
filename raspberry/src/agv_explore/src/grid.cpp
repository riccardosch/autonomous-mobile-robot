#include "agv_explore/grid.hpp"

#include <cmath>

namespace agv_explore {

int Grid::index(double wx, double wy) const {
  const int c = static_cast<int>(std::floor((wx - origin_x) / resolution));
  const int r = static_cast<int>(std::floor((wy - origin_y) / resolution));
  if (c < 0 || r < 0 || c >= width || r >= height) return -1;
  return r * width + c;
}

double distance(const Point &a, const Point &b) { return std::hypot(a.x - b.x, a.y - b.y); }

}  // namespace agv_explore
