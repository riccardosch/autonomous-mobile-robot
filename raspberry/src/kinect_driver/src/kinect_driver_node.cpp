// Publishes the RGB image of a Kinect v1 (Xbox 360) read through libfreenect.
//
// libfreenect delivers frames through C callbacks, called from inside
// freenect_process_events(): an events thread keeps calling it, and the video
// callback turns each frame into a sensor_msgs/Image.

#include <atomic>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include "libfreenect.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace kinect_driver
{

class KinectDriverNode : public rclcpp::Node
{
public:
  KinectDriverNode()
  : Node("kinect")
  {
    // The device index is the order in which the Kinect was plugged in, starting from 0
    const int device_index = declare_parameter("device_index", 0); // The index of the Kinect device to use
    rgb_frame_id_ = declare_parameter("rgb_frame_id", "kinect_rgb_optical_frame"); // The frame_id of the published RGB images

    rgb_pub_ = create_publisher<sensor_msgs::msg::Image>("rgb/image_raw", rclcpp::SensorDataQoS());

    if (freenect_init(&ctx_, nullptr) < 0) {
      throw std::runtime_error("freenect_init failed");
    }
    // Only the camera for now: motor and audio stay free.
    freenect_select_subdevices(ctx_, FREENECT_DEVICE_CAMERA);
    if (freenect_open_device(ctx_, &dev_, device_index) < 0) {
      freenect_shutdown(ctx_);
      throw std::runtime_error("cannot open Kinect #" + std::to_string(device_index));
    }

    // The callbacks are plain C functions: the node is found again through the
    // device "user" pointer.
    freenect_set_user(dev_, this);
    freenect_set_video_callback(dev_, &KinectDriverNode::videoCallback);

    // 640x480 RGB, demosaiced by libfreenect (3 bytes per pixel, about 30 Hz).
    // No buffer is set: libfreenect allocates its own.
    video_mode_ = freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_RGB);
    if (!video_mode_.is_valid || freenect_set_video_mode(dev_, video_mode_) < 0 ||
      freenect_start_video(dev_) < 0)
    {
      closeDevice();
      throw std::runtime_error("cannot start the RGB stream");
    }

    events_thread_ = std::thread([this] {eventsLoop();});

    RCLCPP_INFO(
      get_logger(), "Kinect #%d open, publishing %s (%dx%d) in frame '%s'", device_index,
      rgb_pub_->get_topic_name(), video_mode_.width, video_mode_.height, rgb_frame_id_.c_str());
  }

  ~KinectDriverNode() override
  {
    running_ = false;
    if (events_thread_.joinable()) {
      events_thread_.join();
    }
    freenect_stop_video(dev_);
    closeDevice();
  }

private:
  void eventsLoop()
  {
    while (running_) {
      // The timeout lets the loop notice running_ = false within 100 ms.
      timeval timeout{0, 100000};
      if (freenect_process_events_timeout(ctx_, &timeout) < 0) {
        RCLCPP_ERROR(get_logger(), "freenect_process_events failed, stopping the stream");
        break;
      }
    }
  }

  static void videoCallback(freenect_device * dev, void * rgb, uint32_t /*timestamp*/)
  {
    static_cast<KinectDriverNode *>(freenect_get_user(dev))->publishRgb(rgb);
  }

  void publishRgb(const void * rgb)
  {
    auto msg = std::make_unique<sensor_msgs::msg::Image>();
    // Arrival time on the Pi: the Kinect timestamp counts device clock ticks,
    // not wall time.
    msg->header.stamp = now();
    msg->header.frame_id = rgb_frame_id_;
    msg->width = video_mode_.width;
    msg->height = video_mode_.height;
    msg->encoding = sensor_msgs::image_encodings::RGB8;
    msg->is_bigendian = false;
    msg->step = msg->width * 3;
    const auto * bytes = static_cast<const uint8_t *>(rgb);
    msg->data.assign(bytes, bytes + video_mode_.bytes);
    rgb_pub_->publish(std::move(msg));
  }

  void closeDevice()
  {
    freenect_close_device(dev_);
    freenect_shutdown(ctx_);
  }

  std::string rgb_frame_id_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_pub_;

  freenect_context * ctx_ = nullptr;
  freenect_device * dev_ = nullptr;
  freenect_frame_mode video_mode_{};
  std::thread events_thread_;
  std::atomic<bool> running_{true};
};

}  // namespace kinect_driver

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<kinect_driver::KinectDriverNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("kinect"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
