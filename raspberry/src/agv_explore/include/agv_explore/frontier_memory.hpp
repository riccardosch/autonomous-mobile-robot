// What the explorer remembers about frontiers it has dealt with, without ROS.
// Times are seconds on any monotonic clock, passed in by the caller.
//
// A frontier is identified by its point: two points closer than `radius` are the
// same frontier (the map shifts a little between updates).
//
//   failed (Nav2 aborted, no room, timeout) --> retry after `retry_after_s`, up to
//                                               `max_attempts`, then blacklisted
//   arrived --(after settle_s)--> explored: forgotten
//                             \-> still open, first time: drive up to it next time
//                              \-> still open again (up close): blacklisted
#pragma once

#include <vector>

#include "agv_explore/grid.hpp"

namespace agv_explore {

class FrontierMemory {
 public:
  struct Params {
    double radius = 0.5;          // m, same-frontier distance
    int max_attempts = 3;
    double retry_after_s = 20.0;
    double settle_s = 3.0;        // wait after arriving before judging (SLAM map updates every 2 s)
  };

  // What happened to a frontier, for the caller to report.
  struct Event {
    enum class Kind { Retry, Blacklisted, DriveUpClose };
    Kind kind;
    Point p;
    int attempt = 0;              // for Retry / Blacklisted after failures
  };

  explicit FrontierMemory(const Params &params) : params_(params) {}

  // Nav2 could not reach it, there was no room for the robot, or it took too long.
  Event failed(const Point &p, double now);

  // Nav2 reached the goal for this frontier; judged in review_arrivals once settled.
  void arrived(const Point &p, double now);

  // Judges the arrivals older than settle_s against the current map.
  std::vector<Event> review_arrivals(const Grid &map, double now);

  // Not to be chosen now: blacklisted, waiting for a retry, or arrived and not judged yet.
  bool excluded(const Point &p, double now) const;

  // Seen from the standoff spot and still open: the next goal goes right up to it.
  bool drive_up_close(const Point &p) const;

  // A failed frontier is waiting for another attempt (the exploration is not finished).
  bool waiting_for_retry(double now) const;

  bool arrivals_pending() const { return !arrived_.empty(); }
  size_t blacklisted() const { return blacklist_.size(); }

 private:
  struct Failure { Point p; int count; double last; };
  struct Arrival { Point p; double at; };

  bool near(const std::vector<Point> &list, const Point &p) const;

  Params params_;
  std::vector<Point> blacklist_;
  std::vector<Failure> failures_;
  std::vector<Arrival> arrived_;
  std::vector<Point> up_close_;
};

}  // namespace agv_explore
