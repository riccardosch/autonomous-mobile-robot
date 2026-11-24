#include <gtest/gtest.h>

#include "agv_explore/frontier_memory.hpp"
#include "test_grid.hpp"

using namespace agv_explore;
using Kind = FrontierMemory::Event::Kind;

namespace {
FrontierMemory::Params params() {
  FrontierMemory::Params p;
  p.radius = 0.5;
  p.max_attempts = 3;
  p.retry_after_s = 20.0;
  p.settle_s = 3.0;
  return p;
}
}  // namespace

TEST(FrontierMemory, FailedFrontierWaitsThenIsRetriedThenBlacklisted) {
  FrontierMemory m(params());
  const Point door{1.0, 1.0};
  auto e = m.failed(door, 0.0);
  EXPECT_EQ(e.kind, Kind::Retry);
  EXPECT_EQ(e.attempt, 1);
  EXPECT_TRUE(m.excluded(door, 10.0));          // resting
  EXPECT_TRUE(m.waiting_for_retry(10.0));
  EXPECT_FALSE(m.excluded(door, 21.0));         // may be tried again
  EXPECT_FALSE(m.waiting_for_retry(21.0));
  EXPECT_EQ(m.failed({1.2, 1.1}, 21.0).attempt, 2);   // same frontier (within 0.5 m)
  e = m.failed(door, 45.0);
  EXPECT_EQ(e.kind, Kind::Blacklisted);
  EXPECT_EQ(e.attempt, 3);
  EXPECT_TRUE(m.excluded(door, 1000.0));        // for good
  EXPECT_FALSE(m.waiting_for_retry(50.0));      // a blacklisted one is not waited for
  EXPECT_EQ(m.blacklisted(), 1u);
}

TEST(FrontierMemory, OtherFrontiersAreNotAffected) {
  FrontierMemory m(params());
  m.failed({1.0, 1.0}, 0.0);
  EXPECT_FALSE(m.excluded({3.0, 1.0}, 1.0));
}

TEST(FrontierMemory, ArrivalIsJudgedOnlyAfterTheMapSettles) {
  const Grid open = grid({
    "#######",
    "#.....?",
    "#######",
  });
  FrontierMemory m(params());
  const Point f{5.5, 1.5};
  m.arrived(f, 10.0);
  EXPECT_TRUE(m.excluded(f, 11.0));               // not chosen again before it is judged
  EXPECT_TRUE(m.review_arrivals(open, 12.0).empty());   // 2 s: too early
  EXPECT_TRUE(m.arrivals_pending());

  auto ev = m.review_arrivals(open, 13.5);         // still open after looking at it from afar
  ASSERT_EQ(ev.size(), 1u);
  EXPECT_EQ(ev[0].kind, Kind::DriveUpClose);
  EXPECT_TRUE(m.drive_up_close(f));
  EXPECT_FALSE(m.excluded(f, 13.5));

  m.arrived(f, 20.0);                              // went up to it: still open
  ev = m.review_arrivals(open, 23.5);
  ASSERT_EQ(ev.size(), 1u);
  EXPECT_EQ(ev[0].kind, Kind::Blacklisted);
  EXPECT_TRUE(m.excluded(f, 24.0));
}

TEST(FrontierMemory, ExploredArrivalIsForgotten) {
  Grid seen = grid({
    "#######",
    "#......",
    "#######",
  });
  FrontierMemory m(params());
  m.arrived({5.5, 1.5}, 0.0);
  EXPECT_TRUE(m.review_arrivals(seen, 5.0).empty());
  EXPECT_FALSE(m.arrivals_pending());
  EXPECT_FALSE(m.excluded({5.5, 1.5}, 5.0));
  EXPECT_EQ(m.blacklisted(), 0u);
}
