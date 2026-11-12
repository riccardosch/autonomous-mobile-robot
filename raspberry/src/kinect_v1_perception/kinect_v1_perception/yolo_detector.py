# Copyright 2026 Riccardo Schiavo
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Find objects and people in the Kinect RGB image with YOLO26.

Each image goes through the network, which answers with a list of boxes:
where the object is (pixels), what it is ('person', 'chair', ...) and how
sure it is (score in [0, 1]). The network is slower than the camera, so the
node always works on the newest image and skips the ones in between.

Subscribes (relative names, run the node in the driver's namespace):
  rgb/image_raw       sensor_msgs/Image (rgb8)

Publishes:
  detections          vision_msgs/Detection2DArray  boxes of the last image,
                                                    same header as the image
  detections/image    sensor_msgs/Image (rgb8)      the image with the boxes
                                                    drawn, only while someone
                                                    is subscribed

Needs the pip package ultralytics (in ~/agv_venv) and ros-jazzy-vision-msgs.
"""

import os
import time

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from ultralytics import YOLO
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


class YoloDetector(Node):

    def __init__(self):
        super().__init__('yolo_detector')
        # A .pt file (PyTorch) or an exported folder such as yolo26n_ncnn_model.
        model = os.path.expanduser(
            self.declare_parameter('model', '~/.cache/agv/yolo26n_ncnn_model').value)
        self._min_score = self.declare_parameter('min_score', 0.4).value
        self._image_size = self.declare_parameter('image_size', 640).value
        self._stats_period_s = self.declare_parameter('stats_period_s', 5.0).value

        self._model = YOLO(model, task='detect')
        self._frames = 0
        self._inference_s = 0.0
        self._stats_start = time.monotonic()
        self._last_summary = ''
        self._last_summary_time = 0.0

        self._detections_pub = self.create_publisher(Detection2DArray, 'detections', 10)
        self._image_pub = self.create_publisher(Image, 'detections/image', qos_profile_sensor_data)
        # Sensor QoS (best effort, queue of 1): while the network is busy the
        # older images are dropped, the next callback gets the newest one.
        self.create_subscription(Image, 'rgb/image_raw', self._on_image, qos_profile_sensor_data)
        self.get_logger().info(f'Ready: {os.path.basename(model)}, min score {self._min_score}')

    def _on_image(self, msg):
        if msg.encoding != 'rgb8':
            self.get_logger().error(f'Need rgb8 images, got {msg.encoding}', once=True)
            return
        rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
        rgb = rows[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
        bgr = np.ascontiguousarray(rgb[:, :, ::-1])  # the network wants blue first

        start = time.monotonic()
        result = self._model.predict(
            bgr, imgsz=self._image_size, conf=self._min_score, verbose=False)[0]
        self._inference_s += time.monotonic() - start
        self._frames += 1

        out = Detection2DArray()
        out.header = msg.header  # same stamp and frame as the image
        names = []
        for box in result.boxes:
            x, y, w, h = (float(v) for v in box.xywh[0])  # centre and size, pixels
            name = result.names[int(box.cls[0])]
            detection = Detection2D()
            detection.header = msg.header
            detection.bbox.center.position.x = x
            detection.bbox.center.position.y = y
            detection.bbox.size_x = w
            detection.bbox.size_y = h
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = name
            hypothesis.hypothesis.score = float(box.conf[0])
            detection.results.append(hypothesis)
            out.detections.append(detection)
            names.append(name)
        self._detections_pub.publish(out)

        if self._image_pub.get_subscription_count() > 0:
            drawn = result.plot()[:, :, ::-1]  # BGR with boxes -> RGB
            image = Image()
            image.header = msg.header
            image.height, image.width = drawn.shape[:2]
            image.encoding = 'rgb8'
            image.step = image.width * 3
            image.data = drawn.tobytes()
            self._image_pub.publish(image)

        self._report(names)

    def _report(self, names):
        """Log what is seen when it changes, and the speed every few seconds."""
        summary = ', '.join(f'{names.count(n)} {n}' for n in sorted(set(names))) or 'nothing'
        now = time.monotonic()
        # At most one line per second: a box near the score threshold flickers.
        if summary != self._last_summary and now - self._last_summary_time >= 1.0:
            self._last_summary = summary
            self._last_summary_time = now
            self.get_logger().info(f'seeing: {summary}')
        elapsed = now - self._stats_start
        if self._stats_period_s > 0 and elapsed >= self._stats_period_s:
            self.get_logger().info(
                f'{self._frames / elapsed:.1f} images/s, '
                f'{1e3 * self._inference_s / self._frames:.0f} ms per image')
            self._frames, self._inference_s = 0, 0.0
            self._stats_start = time.monotonic()


def main():
    rclpy.init()
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
