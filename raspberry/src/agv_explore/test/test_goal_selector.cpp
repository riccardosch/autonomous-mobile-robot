#include <gtest/gtest.h>

#include <vector>

#include "agv_explore/goal_selector.hpp"
#include "test_grid.hpp"

using namespace agv_explore;

namespace {
GoalParams params() {
  GoalParams p;
  p.size_weight = 1.0;
  p.turn_weight = 0.0;
  p.standoff = 2.0;
  p.standoff_search = 0.5;
  p.near_search = 2.0;
  p.max_cost = 50;
  return p;
}
FrontierMemory::Params memory_params() { return FrontierMemory::Params{}; }

Frontier frontier(double x, double y, int size, double dist, double nx = 1, double ny = 0) {
  Frontier f;
  f.goal_x = x;
  f.goal_y = y;
  f.size = size;
  f.distance = dist;
  f.nx = nx;
  f.ny = ny;
  return f;
}

// A long free corridor; the cost is 0 everywhere except the column next to the left wall.
struct Corridor {
  Grid map = grid({
    "##########",
    "#.........",
    "#.........",
    "#.........",
    "##########",
  });
  Grid cost = map;
  Corridor() {
    for (int i = 0; i < cost.width * cost.height; ++i) cost.data[i] = map.data[i] == 100 ? 100 : 0;
    for (int r = 1; r <= 3; ++r) cost.data[r * cost.width + 1] = 99;   // robot centre would touch the wall
  }
};
}  // namespace

TEST(GoalSelector, ScorePrefersBigAndClose) {
  GoalParams p = params();
  const Robot r{0, 0, 0};
  EXPECT_GT(frontier_score(frontier(5, 0, 10, 5), r, 1.0, p), frontier_score(frontier(1, 0, 3, 1), r, 1.0, p));
}

TEST(GoalSelector, ScorePrefersWhatIsAhead) {
  GoalParams p = params();
  p.turn_weight = 0.5;
  const Frontier behind = frontier(-2, 0, 4, 2), ahead = frontier(2, 0, 4, 2);
  EXPECT_GT(frontier_score(ahead, {0, 0, 0}, 1.0, p), frontier_score(behind, {0, 0, 0}, 1.0, p));
  EXPECT_GT(frontier_score(behind, {0, 0, 3.14159}, 1.0, p), frontier_score(ahead, {0, 0, 3.14159}, 1.0, p));
}

TEST(GoalSelector, SafeSpotMovesAwayFromTheWall) {
  Corridor c;
  const auto p = safe_spot(c.map, c.cost, {1.5, 2.5}, 2.0, 50);
  ASSERT_TRUE(p);
  EXPECT_DOUBLE_EQ(p->x, 2.5);   // first acceptable cell away from the wall
  EXPECT_DOUBLE_EQ(p->y, 2.5);
  EXPECT_FALSE(safe_spot(c.map, c.cost, {1.5, 2.5}, 0.5, 50));   // too far to reach it
}

TEST(GoalSelector, GoalIsInFrontOfTheFrontierFacingIt) {
  Corridor c;
  const FrontierMemory m(memory_params());
  // Frontier at the open right end (x = 9.5), unknown towards +x.
  const auto s = select_goal({frontier(9.5, 2.5, 3, 7)}, c.map, c.cost, {2.5, 2.5, 0}, m, 0.0, {}, 0.5, params());
  ASSERT_TRUE(s.choice);
  EXPECT_DOUBLE_EQ(s.choice->goal.x, 7.5);   // standoff 2 m before it
  EXPECT_NEAR(s.choice->yaw, 0.0, 1e-9);     // facing +x
  EXPECT_FALSE(s.choice->up_close);
}

TEST(GoalSelector, UpCloseFrontierGetsAGoalNextToIt) {
  Corridor c;
  FrontierMemory m(memory_params());
  const Grid open = grid({"#.?"});
  m.arrived({1.5, 0.5}, 0.0);                     // in `open`, (1.5, 0.5) borders unknown
  m.review_arrivals(open, 10.0);                  // -> drive up close to that point
  const auto s = select_goal({frontier(1.5, 0.5, 3, 1)}, c.map, c.cost, {5.5, 2.5, 0}, m, 10.0, {}, 0.5, params());
  ASSERT_TRUE(s.choice);
  EXPECT_TRUE(s.choice->up_close);
}

TEST(GoalSelector, SkipsExcludedCurrentAndNoRoom) {
  Corridor c;
  FrontierMemory m(memory_params());
  const std::vector<Frontier> f = {
    frontier(9.5, 2.5, 9, 1),      // best score, but being driven to
    frontier(9.5, 1.5, 8, 1),      // next, but failed a moment ago
    frontier(50.0, 50.0, 7, 1),    // outside the maps: no room for the robot
    frontier(5.5, 2.5, 1, 1),      // the one left
  };
  m.failed({9.5, 1.5}, 0.0);
  const auto s = select_goal(f, c.map, c.cost, {2.5, 2.5, 0}, m, 1.0, Point{9.5, 2.5}, 0.5, params());
  ASSERT_TRUE(s.choice);
  EXPECT_DOUBLE_EQ(s.choice->frontier.goal_x, 5.5);
  ASSERT_EQ(s.no_room.size(), 1u);
  EXPECT_DOUBLE_EQ(s.no_room[0].x, 50.0);
}
