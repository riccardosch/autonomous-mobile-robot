#include "agv_explore/frontier_memory.hpp"

#include "agv_explore/frontiers.hpp"

namespace agv_explore {

bool FrontierMemory::near(const std::vector<Point> &list, const Point &p) const {
  for (const auto &q : list)
    if (distance(q, p) < params_.radius) return true;
  return false;
}

FrontierMemory::Event FrontierMemory::failed(const Point &p, double now) {
  for (auto &f : failures_)
    if (distance(f.p, p) < params_.radius) {
      f.last = now;
      if (++f.count >= params_.max_attempts) {
        blacklist_.push_back(p);
        return {Event::Kind::Blacklisted, p, f.count};
      }
      return {Event::Kind::Retry, p, f.count};
    }
  failures_.push_back({p, 1, now});
  if (params_.max_attempts <= 1) {
    blacklist_.push_back(p);
    return {Event::Kind::Blacklisted, p, 1};
  }
  return {Event::Kind::Retry, p, 1};
}

void FrontierMemory::arrived(const Point &p, double now) { arrived_.push_back({p, now}); }

std::vector<FrontierMemory::Event> FrontierMemory::review_arrivals(const Grid &map, double now) {
  std::vector<Event> events;
  for (auto it = arrived_.begin(); it != arrived_.end();) {
    if (now - it->at < params_.settle_s) { ++it; continue; }
    if (still_frontier(map, it->p.x, it->p.y)) {
      if (!near(up_close_, it->p)) {
        up_close_.push_back(it->p);
        events.push_back({Event::Kind::DriveUpClose, it->p});
      } else {
        blacklist_.push_back(it->p);
        events.push_back({Event::Kind::Blacklisted, it->p});
      }
    }
    it = arrived_.erase(it);
  }
  return events;
}

bool FrontierMemory::excluded(const Point &p, double now) const {
  if (near(blacklist_, p)) return true;
  for (const auto &a : arrived_)
    if (distance(a.p, p) < params_.radius) return true;
  for (const auto &f : failures_)
    if (now - f.last < params_.retry_after_s && distance(f.p, p) < params_.radius) return true;
  return false;
}

bool FrontierMemory::drive_up_close(const Point &p) const { return near(up_close_, p); }

bool FrontierMemory::waiting_for_retry(double now) const {
  for (const auto &f : failures_)
    if (f.count < params_.max_attempts && now - f.last < params_.retry_after_s) return true;
  return false;
}

}  // namespace agv_explore
