// Publishes the scans of the ESP32-C3 LiDAR firmware as sensor_msgs/LaserScan.
//
// A reader thread blocks on the serial port, decodes the binary frames and
// publishes each one on /scan. If the port disappears (ESP32 unplugged or
// reset) the thread retries every second.

#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <thread>

#include "lidar_bridge/frame_parser.hpp"
#include "lidar_bridge/serial_port.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace lidar_bridge
{

class LidarBridgeNode : public rclcpp::Node
{
public:
  LidarBridgeNode()
  : Node("lidar_bridge"),
    parser_([this](const ScanFrame & frame) {publishScan(frame);})
  {
    port_ = declare_parameter("port", "/dev/ttyACM0");
    frame_id_ = declare_parameter("frame_id", "laser");
    // YDLIDAR angles grow clockwise seen from above, ROS angles counter-clockwise
    clockwise_ = declare_parameter("clockwise", true);
    angle_offset_ = declare_parameter("angle_offset_deg", 0.0) * M_PI / 180.0;
    range_min_ = declare_parameter("range_min", 0.12);  // X2 datasheet
    range_max_ = declare_parameter("range_max", 8.0);   // X2 datasheet

    publisher_ = create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS());
    reader_ = std::thread([this] {readLoop();});

    RCLCPP_INFO(
      get_logger(), "Reading %s, publishing %s in frame '%s'", port_.c_str(),
      publisher_->get_topic_name(), frame_id_.c_str());
  }

  ~LidarBridgeNode() override
  {
    running_ = false;
    if (reader_.joinable()) {
      reader_.join();
    }
  }

private:
  void readLoop()
  {
    uint8_t buf[1024];
    while (running_ && rclcpp::ok()) {
      if (!serial_.isOpen()) {
        try {
          serial_.open(port_);
          have_seq_ = false;
          skip_next_frame_ = true;
          RCLCPP_INFO(get_logger(), "Opened %s", port_.c_str());
        } catch (const std::exception & e) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "%s, retrying", e.what());
          std::this_thread::sleep_for(std::chrono::seconds(1));
          continue;
        }
      }
      try {
        const size_t n = serial_.read(buf, sizeof(buf), 200);
        if (n > 0) {
          parser_.feed(buf, n);
        }
      } catch (const std::exception & e) {
        RCLCPP_WARN(get_logger(), "%s: %s, reopening", port_.c_str(), e.what());
        serial_.close();
      }
    }
  }

  void publishScan(const ScanFrame & frame)
  {
    // The first frame after opening the port can be an old one, left in the
    // ESP32 USB buffer while nobody was reading: wrong stamp and a fake seq gap.
    if (skip_next_frame_) {
      skip_next_frame_ = false;
      return;
    }
    const rclcpp::Time received = now();
    trackLostFrames(frame.seq);

    const size_t n = frame.distances_mm.size();
    const double scan_time = frame.scan_duration_us * 1e-6;

    auto msg = std::make_unique<sensor_msgs::msg::LaserScan>();
    // The stamp is the time of the first ray: the frame is sent once the whole
    // revolution has been collected (USB latency is ignored).
    msg->header.stamp = received - rclcpp::Duration::from_seconds(scan_time);
    msg->header.frame_id = frame_id_;
    msg->angle_increment = 2.0 * M_PI / n;
    msg->angle_min = angle_offset_;
    msg->angle_max = angle_offset_ + msg->angle_increment * (n - 1);
    msg->scan_time = scan_time;
    msg->time_increment = scan_time / n;
    msg->range_min = range_min_;
    msg->range_max = range_max_;

    msg->ranges.resize(n);
    for (size_t i = 0; i < n; i++) {
      // ranges[i] is at +i*increment counter-clockwise; with a clockwise LiDAR
      // that is device bin -i.
      const size_t bin = clockwise_ ? (n - i) % n : i;
      const uint16_t mm = frame.distances_mm[bin];
      // 0 = no echo: +inf means "nothing within range" (REP 117)
      msg->ranges[i] = mm ? mm * 1e-3f : std::numeric_limits<float>::infinity();
    }
    publisher_->publish(std::move(msg));
  }

  void trackLostFrames(uint16_t seq)
  {
    if (have_seq_) {
      const uint16_t lost = uint16_t(seq - last_seq_ - 1);
      if (lost) {
        lost_frames_ += lost;
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, "Lost %u frames (total %lu), CRC errors %u", lost,
          static_cast<unsigned long>(lost_frames_), parser_.crcErrors());
      }
    }
    last_seq_ = seq;
    have_seq_ = true;
  }

  std::string port_;
  std::string frame_id_;
  bool clockwise_;
  double angle_offset_;
  double range_min_;
  double range_max_;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  SerialPort serial_;
  FrameParser parser_;
  std::thread reader_;
  std::atomic<bool> running_{true};

  bool skip_next_frame_ = true;
  bool have_seq_ = false;
  uint16_t last_seq_ = 0;
  uint64_t lost_frames_ = 0;
};

}  // namespace lidar_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<lidar_bridge::LidarBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
