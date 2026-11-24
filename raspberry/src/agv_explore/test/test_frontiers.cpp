#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "agv_explore/frontiers.hpp"

using namespace agv_explore;

namespace {
// Rows written top to bottom as seen on paper; row 0 of the grid is the last string.
// '.' free, '#' wall, '?' unknown. 1 m cells to make distances easy.
Grid grid(const std::vector<std::string> &rows) {
  Grid g;
  g.height = rows.size();
  g.width = rows[0].size();
  g.resolution = 1.0;
  for (int r = g.height - 1; r >= 0; --r)
    for (char c : rows[r]) g.data.push_back(c == '.' ? 0 : c == '#' ? 100 : -1);
  return g;
}
}  // namespace

TEST(Frontiers, RoomWithOpenDoorHasOneFrontier) {
  const Grid g = grid({
    "#####???",
    "#....???",   // door on the right: free cells touching unknown
    "#....???",
    "#####???",
  });
  const auto f = find_frontiers(g, 1.5, 1.5, 1);
  ASSERT_EQ(f.size(), 1u);
  EXPECT_EQ(f[0].size, 2);
  EXPECT_DOUBLE_EQ(f[0].goal_x, 4.5);   // column 4, the cells next to the unknown
  EXPECT_NEAR(f[0].nx, 1.0, 1e-9);      // the unknown is to the right (+x)
  EXPECT_NEAR(f[0].ny, 0.0, 1e-9);
}

TEST(Frontiers, ClosedRoomHasNone) {
  const Grid g = grid({
    "######",
    "#....#",
    "######",
  });
  EXPECT_TRUE(find_frontiers(g, 1.5, 1.5, 1).empty());
}

TEST(Frontiers, SmallFrontiersAreIgnored) {
  const Grid g = grid({
    "#####",
    "#...?",   // one cell only
    "#####",
  });
  EXPECT_EQ(find_frontiers(g, 1.5, 1.5, 1).size(), 1u);
  EXPECT_TRUE(find_frontiers(g, 1.5, 1.5, 2).empty());
}

TEST(Frontiers, PickPrefersBigAndCloseAndSkipsBlacklist) {
  std::vector<Frontier> f(2);
  f[0].size = 10; f[0].distance = 5; f[0].goal_x = 5; f[0].goal_y = 0;  // ahead: score 10 - 5 = 5
  f[1].size = 3;  f[1].distance = 1; f[1].goal_x = 1; f[1].goal_y = 0;  // ahead: score 3 - 1 = 2
  const Robot r{0, 0, 0};
  EXPECT_EQ(pick_frontier(f, {}, 0.5, 1.0, 1.0, r, 0.0), 0);
  EXPECT_EQ(pick_frontier(f, {{5.2, 0.1}}, 0.5, 1.0, 1.0, r, 0.0), 1);
  EXPECT_EQ(pick_frontier(f, {{5, 0}, {1, 0}}, 0.5, 1.0, 1.0, r, 0.0), -1);
}

TEST(Frontiers, PickPrefersWhatIsAhead) {
  std::vector<Frontier> f(2);
  f[0].size = 4; f[0].distance = 2; f[0].goal_x = -2; f[0].goal_y = 0;  // behind the robot
  f[1].size = 4; f[1].distance = 2; f[1].goal_x = 2;  f[1].goal_y = 0;  // ahead
  EXPECT_EQ(pick_frontier(f, {}, 0.5, 1.0, 1.0, Robot{0, 0, 0}, 0.5), 1);
  EXPECT_EQ(pick_frontier(f, {}, 0.5, 1.0, 1.0, Robot{0, 0, 3.14159}, 0.5), 0);
}

TEST(Frontiers, SafeGoalMovesAwayFromTheWall) {
  // Map: free corridor; costmap: the column next to the wall (x = 0.5) is too close.
  const Grid map = grid({
    "#....",
    "#....",
    "#....",
  });
  Grid cost = map;
  for (int r = 0; r < 3; ++r) {
    cost.data[r * 5 + 0] = 100;   // wall
    cost.data[r * 5 + 1] = 99;    // robot centre would touch it
    cost.data[r * 5 + 2] = 40;
    cost.data[r * 5 + 3] = 0;
    cost.data[r * 5 + 4] = 0;
  }
  Point p{};
  ASSERT_TRUE(safe_goal(map, cost, 1.5, 1.5, 2.0, 50, p));
  EXPECT_DOUBLE_EQ(p.x, 2.5);     // first acceptable cell away from the wall
  EXPECT_DOUBLE_EQ(p.y, 1.5);
  EXPECT_FALSE(safe_goal(map, cost, 1.5, 1.5, 0.5, 50, p));   // too far to reach it
}

TEST(Frontiers, StillFrontierGoesFalseOnceSeen) {
  Grid g = grid({
    "#######",
    "#.....?",
    "#######",
  });
  EXPECT_TRUE(still_frontier(g, 5.5, 1.5));
  g.data[1 * g.width + 6] = 0;   // the LiDAR has now seen that cell
  EXPECT_FALSE(still_frontier(g, 5.5, 1.5));
}
