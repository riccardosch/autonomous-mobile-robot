// explorer: drives the robot to frontiers (edges of the known map) until none is
// left, then saves the map. No goals from the user: it picks them itself.
//
//   /map (slam_toolbox) --> frontiers --> best one --> Nav2 navigate_to_pose
//        ^                                                   |
//        +---------- the LiDAR sees new space there ---------+
//
// This file only talks to ROS (topics, action, service, TF, parameters) and runs
// the state machine; the decisions are in the ROS-free modules:
//   frontiers.hpp        where the frontiers are
//   goal_selector.hpp    which frontier, and where exactly to send the robot
//   frontier_memory.hpp  failures, retries, blacklist, frontiers to approach up close
//   map_file.hpp         the saved map files
//
// States (checked once a second in tick()):
//   WaitingForData --map, costmap, Nav2 ready--> Idle
//   Idle     --goal sent-->                     Driving
//   Driving  --better goal (frontier seen on the way, or timeout)--> Driving (Nav2 replaces it, no stop)
//   Driving  --Nav2 result / goal rejected / nothing else to do--> Idle
//   Idle     --no frontier for empty_checks s, none waiting--> Finished (map saved)
//   any      --time limit--> Finished
#include <chrono>
#include <cmath>
#include <ctime>
#include <memory>
#include <optional>
#include <string>

#include "agv_explore/frontier_memory.hpp"
#include "agv_explore/frontiers.hpp"
#include "agv_explore/goal_selector.hpp"
#include "agv_explore/map_file.hpp"
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
using SerializePoseGraph = slam_toolbox::srv::SerializePoseGraph;

namespace {
Grid to_grid(const nav_msgs::msg::OccupancyGrid &m) {
  Grid g;
  g.width = m.info.width;
  g.height = m.info.height;
  g.resolution = m.info.resolution;
  g.origin_x = m.info.origin.position.x;  // slam_toolbox maps and Nav2 costmaps are not rotated
  g.origin_y = m.info.origin.position.y;
  g.data = m.data;
  return g;
}
}  // namespace

class Explorer : public rclcpp::Node {
 public:
  Explorer()
  : Node("explorer"), memory_params_(load_memory_params()), memory_(memory_params_),
    tf_buffer_(get_clock()), tf_listener_(tf_buffer_) {
    min_size_ = declare_parameter<int>("min_frontier_size", 8);
    goal_.size_weight = declare_parameter<double>("size_weight", goal_.size_weight);
    goal_.turn_weight = declare_parameter<double>("turn_weight", goal_.turn_weight);
    goal_.max_cost = declare_parameter<int>("goal_max_cost", goal_.max_cost);
    goal_.near_search = declare_parameter<double>("goal_search_dist", goal_.near_search);
    goal_.standoff = declare_parameter<double>("standoff", goal_.standoff);
    goal_.standoff_search = declare_parameter<double>("standoff_search_dist", goal_.standoff_search);
    goal_timeout_s_ = declare_parameter<double>("goal_timeout_s", 120.0);
    seen_on_way_after_s_ = declare_parameter<double>("seen_on_way_after_s", 3.0);
    time_limit_s_ = declare_parameter<double>("time_limit_min", 20.0) * 60.0;
    empty_checks_ = declare_parameter<int>("empty_checks", 5);
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
    serialize_ = create_client<SerializePoseGraph>("slam_toolbox/serialize_map");
    start_ = now();
    timer_ = create_wall_timer(std::chrono::seconds(1), [this] { tick(); });
    RCLCPP_INFO(get_logger(), "exploring: waiting for /map, the global costmap and Nav2");
  }

 private:
  enum class State { WaitingForData, Idle, Driving, Finished };

  // Read in the initializer list, before memory_ is built from them.
  FrontierMemory::Params load_memory_params() {
    FrontierMemory::Params p;
    p.radius = declare_parameter<double>("blacklist_radius", p.radius);
    p.max_attempts = declare_parameter<int>("max_attempts", p.max_attempts);
    p.retry_after_s = declare_parameter<double>("retry_after_s", p.retry_after_s);
    p.settle_s = declare_parameter<double>("settle_s", p.settle_s);
    return p;
  }

  double elapsed() const { return (now() - start_).seconds(); }

  void tick() {
    if (state_ == State::Finished) return;
    const double t = elapsed();
    if (t > time_limit_s_) return finish("time limit reached");
    if (!map_ || !costmap_ || !nav_->action_server_is_ready()) return;
    if (state_ == State::WaitingForData) state_ = State::Idle;

    const Grid map = to_grid(*map_);
    const Grid cost = to_grid(*costmap_);
    for (const auto &e : memory_.review_arrivals(map, t)) report(e, "");

    // While driving: keep going, unless the frontier was seen on the way or it takes too long.
    std::optional<Point> current;
    if (state_ == State::Driving) {
      const double driving = t - goal_sent_;
      if (driving > goal_timeout_s_) {
        report(memory_.failed(target_, t), "goal takes too long for the frontier at");
      } else if (driving > seen_on_way_after_s_ && !still_frontier(map, target_.x, target_.y)) {
        RCLCPP_INFO(get_logger(), "frontier at (%.2f, %.2f) seen on the way: next", target_.x, target_.y);
      } else {
        return;
      }
      current = target_;
    }

    Robot robot;
    if (!robot_pose(robot)) return;
    const auto frontiers = find_frontiers(map, robot.x, robot.y, min_size_);
    const auto sel = select_goal(frontiers, map, cost, robot, memory_, t, current,
                                 memory_params_.radius, goal_);
    for (const auto &p : sel.no_room) report(memory_.failed(p, t), "no room for the robot near the frontier at");

    if (sel.choice) {
      empty_ = 0;
      return send_goal(*sel.choice, robot, frontiers.size());
    }
    if (state_ == State::Driving) return stop_driving();  // nothing else to go to: stop here
    // The map may still be catching up, or a frontier waits for another attempt:
    // be sure before declaring the house done.
    if (memory_.waiting_for_retry(t)) { empty_ = 0; return; }
    if (!memory_.arrivals_pending() && ++empty_ >= empty_checks_)
      finish(frontiers.empty() ? "no frontiers left" : "only unreachable frontiers left");
  }

  // A new goal replaces the running one inside Nav2 (preemption): no stop in between.
  void send_goal(const GoalChoice &c, const Robot &robot, size_t n) {
    NavigateToPose::Goal msg;
    msg.pose.header.frame_id = map_->header.frame_id;
    msg.pose.header.stamp = now();
    msg.pose.pose.position.x = c.goal.x;
    msg.pose.pose.position.y = c.goal.y;
    msg.pose.pose.orientation.z = std::sin(c.yaw / 2);
    msg.pose.pose.orientation.w = std::cos(c.yaw / 2);

    target_ = c.frontier.point();
    goal_sent_ = elapsed();
    state_ = State::Driving;
    const int id = ++goal_id_;
    RCLCPP_INFO(get_logger(), "%zu frontiers; going to (%.2f, %.2f)%s for the frontier at (%.2f, %.2f): "
                "%d cells, %.1f m away", n, c.goal.x, c.goal.y, c.up_close ? " up close" : "",
                target_.x, target_.y, c.frontier.size, distance(c.goal, {robot.x, robot.y}));

    rclcpp_action::Client<NavigateToPose>::SendGoalOptions opt;
    opt.goal_response_callback = [this, id](GoalHandle::SharedPtr h) {
      if (id != goal_id_ || h) return;
      report(memory_.failed(target_, elapsed()), "goal rejected by Nav2 for the frontier at");
      state_ = State::Idle;
    };
    opt.result_callback = [this, id](const GoalHandle::WrappedResult &r) { on_result(id, r.code); };
    nav_->async_send_goal(msg, opt);
  }

  void on_result(int id, rclcpp_action::ResultCode code) {
    if (id != goal_id_ || state_ != State::Driving) return;  // a replaced goal
    state_ = State::Idle;
    if (code == rclcpp_action::ResultCode::SUCCEEDED) {
      memory_.arrived(target_, elapsed());  // judged in tick() once the map has caught up
    } else if (code == rclcpp_action::ResultCode::ABORTED) {
      report(memory_.failed(target_, elapsed()), "Nav2 could not reach the frontier at");
    }
  }

  void stop_driving() {
    state_ = State::Idle;
    ++goal_id_;  // ignore the result of the cancelled goal
    nav_->async_cancel_all_goals();
  }

  void finish(const std::string &why) {
    state_ = State::Finished;
    ++goal_id_;  // ignore the result of a running goal
    nav_->async_cancel_all_goals();
    RCLCPP_INFO(get_logger(), "exploration finished (%s) after %.1f min, %zu frontiers blacklisted",
                why.c_str(), elapsed() / 60.0, memory_.blacklisted());
    if (!map_) return;

    const std::string name = map_name_.empty() ? default_map_name(std::time(nullptr)) : map_name_;
    const std::string path = expand_home(map_dir_) + "/" + name;
    // The image map is written here from our own copy of /map (slam_toolbox/save_map
    // runs map_saver, which gives up if /map takes more than 2 s); the pose graph
    // comes from slam_toolbox, to continue the map or localize on it later.
    const bool ok = write_map_files(to_grid(*map_), path);
    RCLCPP_INFO(get_logger(), "map %s: %s.pgm/.yaml", ok ? "saved" : "NOT saved", path.c_str());
    auto req = std::make_shared<SerializePoseGraph::Request>();
    req->filename = path;
    serialize_->async_send_request(req, [this, path](rclcpp::Client<SerializePoseGraph>::SharedFuture f) {
      RCLCPP_INFO(get_logger(), "pose graph %s: %s.posegraph", f.get()->result == 0 ? "saved" : "NOT saved",
                  path.c_str());
    });
  }

  // `context` describes a failure ("Nav2 could not reach the frontier at"); empty for arrivals.
  void report(const FrontierMemory::Event &e, const char *context) {
    using Kind = FrontierMemory::Event::Kind;
    const int max = memory_params_.max_attempts;
    switch (e.kind) {
      case Kind::Retry:
        RCLCPP_WARN(get_logger(), "%s (%.2f, %.2f), attempt %d/%d: retry in %.0f s", context, e.p.x, e.p.y,
                    e.attempt, max, memory_params_.retry_after_s);
        break;
      case Kind::Blacklisted:
        if (e.attempt > 0)
          RCLCPP_WARN(get_logger(), "%s (%.2f, %.2f), attempt %d/%d: blacklisted", context, e.p.x, e.p.y,
                      e.attempt, max);
        else
          RCLCPP_INFO(get_logger(), "frontier at (%.2f, %.2f) still open even up close: blacklisted", e.p.x, e.p.y);
        break;
      case Kind::DriveUpClose:
        RCLCPP_INFO(get_logger(), "frontier at (%.2f, %.2f) still open from %.1f m: next time drive up to it",
                    e.p.x, e.p.y, goal_.standoff);
        break;
    }
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

  // Parameters
  int min_size_ = 8, empty_checks_ = 5;
  GoalParams goal_;
  double goal_timeout_s_ = 120.0, seen_on_way_after_s_ = 3.0, time_limit_s_ = 1200.0;
  std::string map_dir_, map_name_, robot_frame_;

  FrontierMemory::Params memory_params_;

  // State
  State state_ = State::WaitingForData;
  FrontierMemory memory_;
  int empty_ = 0, goal_id_ = 0;
  Point target_{0, 0};      // the frontier being driven to (the goal is a spot near it)
  double goal_sent_ = 0;    // s since start
  rclcpp::Time start_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_, costmap_;

  // ROS
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_, cost_sub_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_;
  rclcpp::Client<SerializePoseGraph>::SharedPtr serialize_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace agv_explore

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<agv_explore::Explorer>());
  rclcpp::shutdown();
  return 0;
}
