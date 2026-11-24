// Frontier search on an occupancy grid, without ROS.
// A frontier cell is a free cell next to an unknown one: the edge of what the
// robot has seen. Neighbouring frontier cells are grouped into frontiers; the
// explorer drives to one, the map grows there, and new frontiers appear.
#pragma once

#include <cstdint>
#include <vector>

namespace agv_explore {

struct Grid {
  int width = 0, height = 0;
  double resolution = 0.05;           // m per cell
  double origin_x = 0, origin_y = 0;  // world position of cell (0, 0)'s corner
  std::vector<int8_t> data;           // row-major: -1 unknown, 0..100 occupancy

  bool free(int i) const { return data[i] >= 0 && data[i] < 50; }
  // Cell index of the world point (x, y), or -1 outside the grid.
  int index(double wx, double wy) const;
  bool unknown(int i) const { return data[i] < 0; }
  double x(int i) const { return origin_x + (i % width + 0.5) * resolution; }
  double y(int i) const { return origin_y + (i / width + 0.5) * resolution; }
};

struct Frontier {
  int size = 0;                    // cells
  double goal_x = 0, goal_y = 0;   // frontier cell closest to the centroid (always free)
  double distance = 0;             // m, straight line from the robot to the goal
  double nx = 0, ny = 0;           // unit vector pointing into the unknown side
};

// All frontiers with at least `min_size` cells.
std::vector<Frontier> find_frontiers(const Grid &g, double robot_x, double robot_y, int min_size);

// True if the cell at (x, y) still borders unknown space (the frontier there is not explored yet).
bool still_frontier(const Grid &g, double x, double y);

struct Point { double x, y; };

// A goal the robot can actually stand on, near the frontier point (x, y): the closest
// cell within max_dist that is free on the map and has cost <= max_cost on the
// costmap (0..100 scale; 99 = robot centre would touch an obstacle). Frontier cells
// are often right next to a wall, where the planner cannot put the robot.
bool safe_goal(const Grid &map, const Grid &costmap, double x, double y, double max_dist,
               int max_cost, Point &out);

// Best frontier. Score = size_weight * length [m] - distance [m] - turn_weight * |turn| [rad],
// where turn is the angle between the robot heading and the direction to the goal:
// close frontiers ahead of the robot win, so it sweeps instead of jumping around.
// Frontiers whose goal is within blacklist_radius of a blacklisted point are skipped.
// Returns -1 if none is left.
struct Robot { double x, y, yaw; };
int pick_frontier(const std::vector<Frontier> &f, const std::vector<Point> &blacklist,
                  double blacklist_radius, double resolution, double size_weight,
                  const Robot &robot, double turn_weight);

}  // namespace agv_explore
