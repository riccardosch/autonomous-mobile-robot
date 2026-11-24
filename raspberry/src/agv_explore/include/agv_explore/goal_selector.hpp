// Which frontier to go to and where exactly to send the robot, without ROS.
#pragma once

#include <optional>
#include <vector>

#include "agv_explore/frontier_memory.hpp"
#include "agv_explore/frontiers.hpp"
#include "agv_explore/grid.hpp"

namespace agv_explore {

struct GoalParams {
  double size_weight = 0.3;        // score = size_weight * length [m] - distance [m] - turn_weight * turn [rad]
  double turn_weight = 0.5;
  double standoff = 0.8;           // m: stop this far in front of the frontier, facing it
  double standoff_search = 0.4;    // m: how far from that point a safe spot may be
  double near_search = 0.8;        // m: otherwise, a safe spot this close to the frontier itself
  int max_cost = 85;               // goal spot cost on the global costmap (0..100; 99 = touching)
};

// Higher is better: close frontiers ahead of the robot win, so it sweeps the house
// instead of jumping around. `resolution` converts the frontier size to metres.
double frontier_score(const Frontier &f, const Robot &robot, double resolution, const GoalParams &p);

// A spot the robot can stand on near (x, y): the closest cell within max_dist that is
// free on the map and has cost <= max_cost on the costmap. Frontier cells are often
// right against a wall, where the planner cannot put the robot.
std::optional<Point> safe_spot(const Grid &map, const Grid &costmap, const Point &near, double max_dist,
                               int max_cost);

struct GoalChoice {
  Frontier frontier;
  Point goal;          // where the robot is sent
  double yaw;          // facing the frontier
  bool up_close;       // goal near the frontier itself, not in front of it
};

struct Selection {
  std::optional<GoalChoice> choice;
  std::vector<Point> no_room;   // frontiers tried and skipped: no safe spot near them
};

// Best frontier first, skipping those the memory excludes and `also_skip` (the one
// being driven to). The goal is `standoff` m in front of the frontier, facing it, so
// the Kinect (0.5-1.5 m) looks into it before the robot drives in; for frontiers the
// memory says to approach up close, or if there is no room in front, a safe spot
// near the frontier itself.
Selection select_goal(const std::vector<Frontier> &frontiers, const Grid &map, const Grid &costmap,
                      const Robot &robot, const FrontierMemory &memory, double now,
                      const std::optional<Point> &also_skip, double same_radius, const GoalParams &p);

}  // namespace agv_explore
