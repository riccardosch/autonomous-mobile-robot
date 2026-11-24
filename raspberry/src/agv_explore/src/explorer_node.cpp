// explorer: drives the robot to frontiers (edges of the known map) until none is
// left, then saves the map. No goals from the user: it picks them itself.
//
//   /map (slam_toolbox) --> frontiers --> best one --> Nav2 navigate_to_pose
//        ^                                                   |
//        +---------- the LiDAR sees new space there ---------+
//
// The goal is a spot near the frontier where the robot fits (global costmap), and a
// new goal replaces the running one without stopping. A frontier that Nav2 cannot
// reach, with no room near it, or still open a few seconds after arriving (space
// the LiDAR cannot see into) goes on a blacklist.
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "agv_explore/frontiers.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "slam_toolbox/srv/serialize_pose_graph.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace agv_explore {

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class Explorer : public rclcpp::Node {
 public:
  Explorer() : Node("explorer"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_) {
    min_size_ = declare_parameter<int>("min_frontier_size", 8);
    size_weight_ = declare_parameter<double>("size_weight", 0.3);
    turn_weight_ = declare_parameter<double>("turn_weight", 0.5);
    goal_max_cost_ = declare_parameter<int>("goal_max_cost", 85);
    goal_search_dist_ = declare_parameter<double>("goal_search_dist", 0.8);
    standoff_ = declare_parameter<double>("standoff", 0.8);
    max_attempts_ = declare_parameter<int>("max_attempts", 3);
    retry_after_s_ = declare_parameter<double>("retry_after_s", 20.0);
    blacklist_radius_ = declare_parameter<double>("blacklist_radius", 0.5);
    goal_timeout_s_ = declare_parameter<double>("goal_timeout_s", 120.0);
    time_limit_s_ = declare_parameter<double>("time_limit_min", 20.0) * 60.0;
    empty_checks_ = declare_parameter<int>("empty_checks", 5);
    settle_s_ = declare_parameter<double>("settle_s", 3.0);
    map_dir_ = declare_parameter<std::string>("map_dir", "~/maps");
    map_name_ = declare_parameter<std::string>("map_name", "");
    robot_frame_ = declare_parameter<std::string>("robot_frame", "base_link");

    // Both are latched (transient local): we get the last one even if we start late.
    const auto latched = rclcpp::QoS(1).transient_local().reliable();
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", latched, [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m) { map_ = m; });
    cost_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "global_costmap/costmap", latched,
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr m) { costmap_ = m; });
    nav_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    serialize_ = create_client<slam_toolbox::srv::SerializePoseGraph>("slam_toolbox/serialize_map");
    start_ = now();
    timer_ = create_wall_timer(std::chrono::seconds(1), [this] { tick(); });
    RCLCPP_INFO(get_logger(), "exploring: waiting for /map, the global costmap and Nav2");
  }

 private:
  enum class State { Idle, Driving, Done };
  struct Pending { Point p; rclcpp::Time at; };
  struct Failed { Point p; int count; rclcpp::Time last; };

  // A frontier that could not be reached is tried again later (the costmap changes:
  // old marks fade, the map improves); only after max_attempts it is blacklisted.
  void failed(const Point &p, const char *why) {
    for (auto &f : failed_)
      if (std::hypot(f.p.x - p.x, f.p.y - p.y) < blacklist_radius_) {
        f.last = now();
        if (++f.count >= max_attempts_) {
          RCLCPP_WARN(get_logger(), "%s (%.2f, %.2f), attempt %d/%d: blacklisted", why, p.x, p.y, f.count, max_attempts_);
          blacklist_.push_back(p);
        } else {
          RCLCPP_WARN(get_logger(), "%s (%.2f, %.2f), attempt %d/%d: retry in %.0f s", why, p.x, p.y, f.count,
                      max_attempts_, retry_after_s_);
        }
        return;
      }
    failed_.push_back({p, 1, now()});
    RCLCPP_WARN(get_logger(), "%s (%.2f, %.2f), attempt 1/%d: retry in %.0f s", why, p.x, p.y, max_attempts_,
                retry_after_s_);
    if (max_attempts_ <= 1) blacklist_.push_back(p);
  }

  bool near(const std::vector<Point> &list, const Point &p) const {
    for (const auto &q : list)
      if (std::hypot(q.x - p.x, q.y - p.y) < blacklist_radius_) return true;
    return false;
  }

  bool cooling_down() const {
    for (const auto &f : failed_)
      if (f.count < max_attempts_ && (now() - f.last).seconds() < retry_after_s_) return true;
    return false;
  }

  void tick() {
    if (state_ == State::Done) return;
    if ((now() - start_).seconds() > time_limit_s_) return finish("time limit reached");
    if (!map_ || !costmap_ || !nav_->action_server_is_ready()) return;
    const Grid map = to_grid(*map_);
    const Grid cost = to_grid(*costmap_);

    // Frontiers we arrived at: once SLAM has added what the robot sees from there
    // (map every 2 s), a frontier still open means the LiDAR cannot see into it
    // (glass, a gap under furniture): blacklist it, or we would come back forever.
    for (auto it = arrived_.begin(); it != arrived_.end();) {
      if ((now() - it->at).seconds() < settle_s_) { ++it; continue; }
      if (still_frontier(map, it->p.x, it->p.y)) {
        if (!near(close_, it->p)) {
          // Seen from the standoff spot and still open (something hides it): go right up to it.
          RCLCPP_INFO(get_logger(), "frontier at (%.2f, %.2f) still open from %.1f m: next time drive up to it",
                      it->p.x, it->p.y, standoff_);
          close_.push_back(it->p);
        } else {
          RCLCPP_INFO(get_logger(), "frontier at (%.2f, %.2f) still open even up close: blacklisted",
                      it->p.x, it->p.y);
          blacklist_.push_back(it->p);
        }
      }
      it = arrived_.erase(it);
    }

    if (state_ == State::Driving) {
      const double driving = (now() - goal_sent_).seconds();
      if (driving > goal_timeout_s_) {
        failed(target_, "goal takes too long for the frontier at");
      } else if (driving > 3.0 && !still_frontier(map, target_.x, target_.y)) {
        RCLCPP_INFO(get_logger(), "frontier at (%.2f, %.2f) seen on the way: next", target_.x, target_.y);
      } else {
        return;  // keep going
      }
    }

    Robot robot;
    if (!robot_pose(robot)) return;
    const auto frontiers = find_frontiers(map, robot.x, robot.y, min_size_);

    // Best frontier first; skip those with no safe spot near them.
    std::vector<Point> skip = blacklist_;
    for (const auto &a : arrived_) skip.push_back(a.p);  // until the map has caught up
    if (state_ == State::Driving) skip.push_back(target_);
    for (const auto &f : failed_)
      if ((now() - f.last).seconds() < retry_after_s_) skip.push_back(f.p);
    for (;;) {
      const int k = pick_frontier(frontiers, skip, blacklist_radius_, map.resolution, size_weight_,
                                  robot, turn_weight_);
      if (k < 0) break;
      const Frontier &f = frontiers[k];
      const Point frontier{f.goal_x, f.goal_y};
      // Stop `standoff` m in front of the frontier, facing it: the Kinect (0.5-1.5 m)
      // then looks straight into the gap before the robot drives in. If there is no
      // room there, any safe spot near the frontier itself.
      // A frontier that stayed open when seen from the standoff: drive up to it instead.
      Point goal;
      const bool front = !near(close_, frontier) &&
                         safe_goal(map, cost, f.goal_x - f.nx * standoff_, f.goal_y - f.ny * standoff_,
                                   0.4, goal_max_cost_, goal);
      if (!front && !safe_goal(map, cost, frontier.x, frontier.y, goal_search_dist_, goal_max_cost_, goal)) {
        failed(frontier, "no room for the robot near the frontier at");
        skip.push_back(frontier);
        continue;
      }
      empty_ = 0;
      return send_goal(frontier, goal, robot, f, frontiers.size());
    }

    if (state_ == State::Driving) {
      // Current target seen or given up and nothing else to go to: stop the robot here.
      state_ = State::Idle;
      ++goal_id_;
      nav_->async_cancel_all_goals();
      return;
    }
    // The map may still be catching up, or a frontier waits for another attempt:
    // be sure before declaring the house done.
    if (cooling_down()) { empty_ = 0; return; }
    if (arrived_.empty() && ++empty_ >= empty_checks_)
      finish(frontiers.empty() ? "no frontiers left" : "only unreachable frontiers left");
  }

  // A new goal replaces the running one inside Nav2 (preemption): no stop in between.
  void send_goal(const Point &frontier, const Point &goal, const Robot &robot, const Frontier &f, size_t n) {
    NavigateToPose::Goal msg;
    msg.pose.header.frame_id = map_->header.frame_id;
    msg.pose.header.stamp = now();
    msg.pose.pose.position.x = goal.x;
    msg.pose.pose.position.y = goal.y;
    const double yaw = std::atan2(frontier.y - goal.y, frontier.x - goal.x);  // face the frontier
    msg.pose.pose.orientation.z = std::sin(yaw / 2);
    msg.pose.pose.orientation.w = std::cos(yaw / 2);

    target_ = frontier;
    goal_sent_ = now();
    state_ = State::Driving;
    const int id = ++goal_id_;
    RCLCPP_INFO(get_logger(), "%zu frontiers; going to (%.2f, %.2f) for the frontier at (%.2f, %.2f): "
                "%d cells, %.1f m away", n, goal.x, goal.y, frontier.x, frontier.y, f.size,
                std::hypot(goal.x - robot.x, goal.y - robot.y));

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions opt;
    opt.goal_response_callback = [this, id](GoalHandle::SharedPtr h) {
      if (id != goal_id_) return;
      if (!h) {
        failed(target_, "goal rejected by Nav2 for the frontier at");
        state_ = State::Idle;
      }
    };
    opt.result_callback = [this, id](const GoalHandle::WrappedResult &r) { on_result(id, r.code); };
    nav_->async_send_goal(msg, opt);
  }

  void on_result(int id, rclcpp_action::ResultCode code) {
    if (id != goal_id_ || state_ != State::Driving) return;  // a replaced goal
    state_ = State::Idle;
    if (code == rclcpp_action::ResultCode::SUCCEEDED) {
      arrived_.push_back({target_, now()});  // checked in tick() once the map has caught up
    } else if (code == rclcpp_action::ResultCode::ABORTED) {
      failed(target_, "Nav2 could not reach the frontier at");
    }
  }

  void finish(const std::string &why) {
    state_ = State::Done;
    ++goal_id_;
    nav_->async_cancel_all_goals();
    const double minutes = (now() - start_).seconds() / 60.0;
    RCLCPP_INFO(get_logger(), "exploration finished (%s) after %.1f min, %zu frontiers blacklisted",
                why.c_str(), minutes, blacklist_.size());

    std::string dir = map_dir_;
    if (dir.rfind("~", 0) == 0) dir = std::string(std::getenv("HOME")) + dir.substr(1);
    std::filesystem::create_directories(dir);
    std::string name = map_name_;
    if (name.empty()) {
      char buf[32];
      const std::time_t t = std::time(nullptr);
      std::strftime(buf, sizeof buf, "house_%Y%m%d_%H%M", std::localtime(&t));
      name = buf;
    }
    const std::string path = dir + "/" + name;

    // Image map (.pgm + .yaml) for Nav2, written here from our own copy of /map
    // (slam_toolbox/save_map runs map_saver, which gives up if /map takes > 2 s),
    // and slam_toolbox's pose graph (.posegraph + .data) to continue or localize later.
    RCLCPP_INFO(get_logger(), "map %s: %s.pgm/.yaml", write_map(*map_, path) ? "saved" : "NOT saved", path.c_str());
    auto ser = std::make_shared<slam_toolbox::srv::SerializePoseGraph::Request>();
    ser->filename = path;
    serialize_->async_send_request(ser, [this, path](rclcpp::Client<slam_toolbox::srv::SerializePoseGraph>::SharedFuture f) {
      RCLCPP_INFO(get_logger(), "pose graph %s: %s.posegraph", f.get()->result == 0 ? "saved" : "NOT saved", path.c_str());
    });
  }

  // Same format as nav2_map_server's map_saver (trinary): free 254, occupied 0, unknown 205.
  static bool write_map(const nav_msgs::msg::OccupancyGrid &m, const std::string &path) {
    const int w = m.info.width, h = m.info.height;
    std::ofstream pgm(path + ".pgm", std::ios::binary);
    pgm << "P5\n# agv_explore\n" << w << " " << h << "\n255\n";
    for (int r = h - 1; r >= 0; --r)  // image top row = largest y
      for (int c = 0; c < w; ++c) {
        const int8_t v = m.data[r * w + c];
        pgm.put(static_cast<char>(v < 0 ? 205 : v >= 65 ? 0 : v <= 25 ? 254 : 205));
      }
    std::ofstream yaml(path + ".yaml");
    const std::string image = std::filesystem::path(path).filename().string() + ".pgm";
    yaml << "image: " << image << "\nmode: trinary\nresolution: " << m.info.resolution
         << "\norigin: [" << m.info.origin.position.x << ", " << m.info.origin.position.y
         << ", 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n";
    return pgm.good() && yaml.good();
  }

  bool robot_pose(Robot &r) {
    try {
      const auto t = tf_buffer_.lookupTransform(map_->header.frame_id, robot_frame_, tf2::TimePointZero);
      const auto &q = t.transform.rotation;
      r.x = t.transform.translation.x;
      r.y = t.transform.translation.y;
      r.yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
      return true;
    } catch (const tf2::TransformException &e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "no robot pose: %s", e.what());
      return false;
    }
  }

  static Grid to_grid(const nav_msgs::msg::OccupancyGrid &m) {
    Grid g;
    g.width = m.info.width;
    g.height = m.info.height;
    g.resolution = m.info.resolution;
    g.origin_x = m.info.origin.position.x;  // slam_toolbox maps and Nav2 costmaps are not rotated
    g.origin_y = m.info.origin.position.y;
    g.data = m.data;
    return g;
  }

  int min_size_, goal_max_cost_, empty_checks_, max_attempts_;
  double retry_after_s_;
  double size_weight_, turn_weight_, goal_search_dist_, standoff_, blacklist_radius_, goal_timeout_s_,
    time_limit_s_, settle_s_;
  std::string map_dir_, map_name_, robot_frame_;

  State state_ = State::Idle;
  int empty_ = 0, goal_id_ = 0;
  Point target_{0, 0};           // the frontier we are going to (the goal is a safe spot near it)
  std::vector<Point> blacklist_;
  std::vector<Pending> arrived_;
  std::vector<Failed> failed_;
  std::vector<Point> close_;     // frontiers to approach up close (still open from the standoff)
  rclcpp::Time start_, goal_sent_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_, costmap_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_, cost_sub_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_;
  rclcpp::Client<slam_toolbox::srv::SerializePoseGraph>::SharedPtr serialize_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace agv_explore

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<agv_explore::Explorer>());
  rclcpp::shutdown();
  return 0;
}
