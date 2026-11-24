#include "agv_explore/frontiers.hpp"

#include <cmath>
#include <limits>

namespace agv_explore {

int Grid::index(double wx, double wy) const {
  const int c = static_cast<int>(std::floor((wx - origin_x) / resolution));
  const int r = static_cast<int>(std::floor((wy - origin_y) / resolution));
  if (c < 0 || r < 0 || c >= width || r >= height) return -1;
  return r * width + c;
}

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

bool safe_goal(const Grid &map, const Grid &costmap, double x, double y, double max_dist,
               int max_cost, Point &out) {
  bool found = false;
  double best = max_dist;
  const int reach = static_cast<int>(std::ceil(max_dist / costmap.resolution));
  const int centre = costmap.index(x, y);
  if (centre < 0) return false;
  const int c0 = centre % costmap.width, r0 = centre / costmap.width;
  for (int r = r0 - reach; r <= r0 + reach; ++r)
    for (int c = c0 - reach; c <= c0 + reach; ++c) {
      if (c < 0 || r < 0 || c >= costmap.width || r >= costmap.height) continue;
      const int i = r * costmap.width + c;
      const int8_t cost = costmap.data[i];
      if (cost < 0 || cost > max_cost) continue;
      const double cx = costmap.x(i), cy = costmap.y(i);
      const int m = map.index(cx, cy);
      if (m < 0 || !map.free(m)) continue;
      const double d = std::hypot(cx - x, cy - y);
      if (d <= best) { best = d; out = {cx, cy}; found = true; }
    }
  return found;
}

int pick_frontier(const std::vector<Frontier> &f, const std::vector<Point> &blacklist,
                  double blacklist_radius, double resolution, double size_weight,
                  const Robot &robot, double turn_weight) {
  int best = -1;
  double best_score = -std::numeric_limits<double>::max();
  for (size_t k = 0; k < f.size(); ++k) {
    bool banned = false;
    for (const auto &b : blacklist)
      if (std::hypot(f[k].goal_x - b.x, f[k].goal_y - b.y) < blacklist_radius) { banned = true; break; }
    if (banned) continue;
    double turn = std::atan2(f[k].goal_y - robot.y, f[k].goal_x - robot.x) - robot.yaw;
    turn = std::fabs(std::atan2(std::sin(turn), std::cos(turn)));  // wrap to [0, pi]
    const double score = size_weight * f[k].size * resolution - f[k].distance - turn_weight * turn;
    if (score > best_score) { best_score = score; best = static_cast<int>(k); }
  }
  return best;
}

}  // namespace agv_explore
