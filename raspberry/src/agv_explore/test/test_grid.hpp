// Test helper: a grid drawn as text. Rows top to bottom as seen on paper (row 0 of
// the grid is the last string). '.' free, '#' wall, '?' unknown. 1 m cells.
#pragma once

#include <string>
#include <vector>

#include "agv_explore/grid.hpp"

inline agv_explore::Grid grid(const std::vector<std::string> &rows) {
  agv_explore::Grid g;
  g.height = static_cast<int>(rows.size());
  g.width = static_cast<int>(rows[0].size());
  g.resolution = 1.0;
  for (int r = g.height - 1; r >= 0; --r)
    for (char c : rows[r]) g.data.push_back(c == '.' ? 0 : c == '#' ? 100 : -1);
  return g;
}
