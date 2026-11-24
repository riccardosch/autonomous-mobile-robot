#include "agv_explore/goal_selector.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace agv_explore {

double frontier_score(const Frontier &f, const Robot &robot, double resolution, const GoalParams &p) {
  double turn = std::atan2(f.goal_y - robot.y, f.goal_x - robot.x) - robot.yaw;
  turn = std::fabs(std::atan2(std::sin(turn), std::cos(turn)));  // wrap to [0, pi]
  return p.size_weight * f.size * resolution - f.distance - p.turn_weight * turn;
}

std::optional<Point> safe_spot(const Grid &map, const Grid &costmap, const Point &near, double max_dist,
                               int max_cost) {
  std::optional<Point> out;
  double best = max_dist;
  const int reach = static_cast<int>(std::ceil(max_dist / costmap.resolution));
  const int centre = costmap.index(near.x, near.y);
  if (centre < 0) return out;
  const int c0 = centre % costmap.width, r0 = centre / costmap.width;
  for (int r = r0 - reach; r <= r0 + reach; ++r)
    for (int c = c0 - reach; c <= c0 + reach; ++c) {
      if (c < 0 || r < 0 || c >= costmap.width || r >= costmap.height) continue;
      const int i = r * costmap.width + c;
      const int8_t cost = costmap.data[i];
      if (cost < 0 || cost > max_cost) continue;
      const Point cell{costmap.x(i), costmap.y(i)};
      const int m = map.index(cell.x, cell.y);
      if (m < 0 || !map.free(m)) continue;
      const double d = distance(cell, near);
      if (d <= best) { best = d; out = cell; }
    }
  return out;
}

Selection select_goal(const std::vector<Frontier> &frontiers, const Grid &map, const Grid &costmap,
                      const Robot &robot, const FrontierMemory &memory, double now,
                      const std::optional<Point> &also_skip, double same_radius, const GoalParams &p) {
  // Best score first; on equal scores the earlier frontier wins.
  std::vector<size_t> order(frontiers.size());
  std::iota(order.begin(), order.end(), 0);
  std::vector<double> score(frontiers.size());
  for (size_t k = 0; k < frontiers.size(); ++k) score[k] = frontier_score(frontiers[k], robot, map.resolution, p);
  std::stable_sort(order.begin(), order.end(), [&](size_t a, size_t b) { return score[a] > score[b]; });

  Selection s;
  for (size_t k : order) {
    const Frontier &f = frontiers[k];
    const Point fp = f.point();
    if (memory.excluded(fp, now)) continue;
    if (also_skip && distance(*also_skip, fp) < same_radius) continue;
    const bool next_to_no_room = std::any_of(s.no_room.begin(), s.no_room.end(),
                                             [&](const Point &q) { return distance(q, fp) < same_radius; });
    if (next_to_no_room) continue;

    const bool up_close = memory.drive_up_close(fp);
    std::optional<Point> goal;
    if (!up_close)
      goal = safe_spot(map, costmap, {f.goal_x - f.nx * p.standoff, f.goal_y - f.ny * p.standoff},
                       p.standoff_search, p.max_cost);
    const bool in_front = goal.has_value();
    if (!goal) goal = safe_spot(map, costmap, fp, p.near_search, p.max_cost);
    if (!goal) {
      s.no_room.push_back(fp);
      continue;
    }
    s.choice = GoalChoice{f, *goal, std::atan2(fp.y - goal->y, fp.x - goal->x), !in_front};
    return s;
  }
  return s;
}

}  // namespace agv_explore
