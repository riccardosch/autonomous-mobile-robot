// depth_obstacles: Kinect depth image -> two small point clouds in base_footprint
// for the Nav2 costmaps:
//   ~/points  obstacles between min_height and max_height (marking)
//   ~/floor   floor points (clearing: rays to the floor erase obstacles that left)
// Using one pixel in `step`^2 and publishing at most `rate_hz` keeps the clouds
// small; the full 640x480 cloud at 30 Hz is too much for the costmaps on the Pi.
#include <memory>
#include <string>

#include "agv_perception_depth/depth_to_points.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace agv_perception_depth {

class DepthObstaclesNode : public rclcpp::Node {
 public:
  DepthObstaclesNode() : Node("depth_obstacles"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_) {
    frame_ = declare_parameter<std::string>("target_frame", "base_footprint");
    params_.step = declare_parameter<int>("step", params_.step);
    params_.min_range = declare_parameter<double>("min_range", params_.min_range);
    params_.max_range = declare_parameter<double>("max_range", params_.max_range);
    params_.min_height = declare_parameter<double>("min_height", params_.min_height);
    params_.max_height = declare_parameter<double>("max_height", params_.max_height);
    params_.floor_band = declare_parameter<double>("floor_band", params_.floor_band);
    const double rate = declare_parameter<double>("rate_hz", 10.0);
    min_period_ = rclcpp::Duration::from_seconds(rate > 0.0 ? 1.0 / rate : 0.0);

    // The driver publishes best effort: subscribe the same way, or nothing arrives.
    const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(1);
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "depth/camera_info", sensor_qos, [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr m) {
        if (m->k[0] > 0.0) k_ = Intrinsics{m->k[0], m->k[4], m->k[2], m->k[5]}, have_k_ = true;
      });
    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "depth/image_raw", sensor_qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr m) { on_image(*m); });
    points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/points", rclcpp::SensorDataQoS());
    floor_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("~/floor", rclcpp::SensorDataQoS());
  }

 private:
  void on_image(const sensor_msgs::msg::Image &img) {
    const rclcpp::Time stamp(img.header.stamp);
    if (last_.nanoseconds() != 0 && stamp - last_ < min_period_) return;
    if (!have_k_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "waiting for depth/camera_info");
      return;
    }
    if (img.encoding != "16UC1" || img.step != img.width * 2) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                            "expected a packed 16UC1 depth image in mm, got %s", img.encoding.c_str());
      return;
    }
    // The camera is fixed on the robot: the latest transform is the right one.
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_.lookupTransform(frame_, img.header.frame_id, tf2::TimePointZero);
    } catch (const tf2::TransformException &e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "no TF %s -> %s: %s",
                           frame_.c_str(), img.header.frame_id.c_str(), e.what());
      return;
    }
    last_ = stamp;

    const auto &q = tf.transform.rotation;
    const tf2::Matrix3x3 m(tf2::Quaternion(q.x, q.y, q.z, q.w));
    Pose pose{};
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c) pose.R[r * 3 + c] = m[r][c];
    pose.t[0] = tf.transform.translation.x;
    pose.t[1] = tf.transform.translation.y;
    pose.t[2] = tf.transform.translation.z;

    const auto *depth = reinterpret_cast<const uint16_t *>(img.data.data());
    const Result r = depth_to_points(depth, img.width, img.height, k_, pose, params_);
    points_pub_->publish(to_cloud(r.obstacles, img.header.stamp));
    floor_pub_->publish(to_cloud(r.floor, img.header.stamp));
  }

  sensor_msgs::msg::PointCloud2 to_cloud(const std::vector<Point> &pts,
                                         const builtin_interfaces::msg::Time &stamp) const {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.stamp = stamp;
    cloud.header.frame_id = frame_;
    sensor_msgs::PointCloud2Modifier mod(cloud);
    mod.setPointCloud2FieldsByString(1, "xyz");
    mod.resize(pts.size());
    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x"), y(cloud, "y"), z(cloud, "z");
    for (const auto &p : pts) { *x = p.x; *y = p.y; *z = p.z; ++x; ++y; ++z; }
    return cloud;
  }

  std::string frame_;
  Params params_;
  Intrinsics k_{};
  bool have_k_ = false;
  rclcpp::Time last_{0, 0, RCL_ROS_TIME};
  rclcpp::Duration min_period_{0, 0};
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub_, floor_pub_;
};

}  // namespace agv_perception_depth

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<agv_perception_depth::DepthObstaclesNode>());
  rclcpp::shutdown();
  return 0;
}
