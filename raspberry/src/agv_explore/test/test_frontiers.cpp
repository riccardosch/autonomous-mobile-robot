#include <gtest/gtest.h>

#include "agv_explore/frontiers.hpp"
#include "test_grid.hpp"

using namespace agv_explore;

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
