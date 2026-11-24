#include "agv_explore/frontiers.hpp"

#include <cmath>
#include <limits>

namespace agv_explore {

namespace {
// Free cell with an unknown cell among its 4 neighbours.
bool is_frontier_cell(const Grid &g, int i) {
  if (!g.free(i)) return false;
  const int c = i % g.width, r = i / g.width;
  return (c > 0 && g.unknown(i - 1)) || (c + 1 < g.width && g.unknown(i + 1)) ||
         (r > 0 && g.unknown(i - g.width)) || (r + 1 < g.height && g.unknown(i + g.width));
}
}  // namespace

std::vector<Frontier> find_frontiers(const Grid &g, double robot_x, double robot_y, int min_size) {
  std::vector<Frontier> out;
  const int n = g.width * g.height;
  std::vector<bool> seen(n, false);
  std::vector<int> cluster, todo;

  for (int start = 0; start < n; ++start) {
    if (seen[start] || !is_frontier_cell(g, start)) continue;
    // Grow one frontier: all frontier cells 8-connected to `start` (flood fill).
    cluster.clear();
    todo.assign(1, start);
    seen[start] = true;
    while (!todo.empty()) {
      const int i = todo.back();
      todo.pop_back();
      cluster.push_back(i);
      const int c = i % g.width, r = i / g.width;
      for (int dr = -1; dr <= 1; ++dr)
        for (int dc = -1; dc <= 1; ++dc) {
          const int cc = c + dc, rr = r + dr;
          if (cc < 0 || rr < 0 || cc >= g.width || rr >= g.height) continue;
          const int j = rr * g.width + cc;
          if (!seen[j] && is_frontier_cell(g, j)) { seen[j] = true; todo.push_back(j); }
        }
    }
    if (static_cast<int>(cluster.size()) < min_size) continue;

    // Goal: the cluster cell closest to its centroid (the centroid itself may be in a wall).
    // Normal: sum of the steps from each frontier cell to its unknown neighbours.
    double cx = 0, cy = 0, nx = 0, ny = 0;
    for (int i : cluster) {
      cx += g.x(i);
      cy += g.y(i);
      const int c = i % g.width, r = i / g.width;
      if (c > 0 && g.unknown(i - 1)) nx -= 1;
      if (c + 1 < g.width && g.unknown(i + 1)) nx += 1;
      if (r > 0 && g.unknown(i - g.width)) ny -= 1;
      if (r + 1 < g.height && g.unknown(i + g.width)) ny += 1;
    }
    cx /= cluster.size();
    cy /= cluster.size();
    int best = cluster[0];
    double best_d = std::numeric_limits<double>::max();
    for (int i : cluster) {
      const double d = std::hypot(g.x(i) - cx, g.y(i) - cy);
      if (d < best_d) { best_d = d; best = i; }
    }
    Frontier f;
    f.size = static_cast<int>(cluster.size());
    f.goal_x = g.x(best);
    f.goal_y = g.y(best);
    f.distance = std::hypot(f.goal_x - robot_x, f.goal_y - robot_y);
    double n = std::hypot(nx, ny);
    if (n < 1e-9) {  // unknown on all sides evenly: face it from where the robot is
      nx = f.goal_x - robot_x;
      ny = f.goal_y - robot_y;
      n = std::hypot(nx, ny);
    }
    if (n > 1e-9) { f.nx = nx / n; f.ny = ny / n; }
    out.push_back(f);
  }
  return out;
}

bool still_frontier(const Grid &g, double x, double y) {
  const int c = static_cast<int>(std::floor((x - g.origin_x) / g.resolution));
  const int r = static_cast<int>(std::floor((y - g.origin_y) / g.resolution));
  // Look 2 cells around: the map shifts a little between updates.
  for (int dr = -2; dr <= 2; ++dr)
    for (int dc = -2; dc <= 2; ++dc) {
      const int cc = c + dc, rr = r + dr;
      if (cc < 0 || rr < 0 || cc >= g.width || rr >= g.height) continue;
      if (is_frontier_cell(g, rr * g.width + cc)) return true;
    }
  return false;
}

}  // namespace agv_explore
