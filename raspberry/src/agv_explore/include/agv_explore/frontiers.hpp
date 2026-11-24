// Frontier search on an occupancy grid, without ROS.
// A frontier cell is a free cell next to an unknown one: the edge of what the
// robot has seen. Neighbouring frontier cells are grouped into frontiers; the
// explorer drives to one, the map grows there, and new frontiers appear.
#pragma once

#include <vector>

#include "agv_explore/grid.hpp"

namespace agv_explore {

struct Frontier {
  int size = 0;                    // cells
  double goal_x = 0, goal_y = 0;   // frontier cell closest to the centroid (always free)
  double distance = 0;             // m, straight line from the robot to that cell
  double nx = 0, ny = 0;           // unit vector pointing into the unknown side

  Point point() const { return {goal_x, goal_y}; }
};

// All frontiers with at least `min_size` cells.
std::vector<Frontier> find_frontiers(const Grid &g, double robot_x, double robot_y, int min_size);

// True if the cell at (x, y), or one within 2 cells, still borders unknown space
// (the frontier there is not explored yet; the map shifts a little between updates).
bool still_frontier(const Grid &g, double x, double y);

}  // namespace agv_explore
