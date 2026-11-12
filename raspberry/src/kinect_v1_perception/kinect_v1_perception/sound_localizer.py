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
Detect sounds on the Kinect microphone array and estimate where they come from.

Subscribes (relative names, run the node in the driver's namespace):
  audio/info   audio_common_msgs/AudioInfo         format of the stream (latched)
  audio/data   audio_common_msgs/AudioDataStamped  interleaved PCM chunks

Publishes:
  sound/detected   std_msgs/Bool              True while a sound is heard (latched,
                                              published on change only)
  sound/direction  geometry_msgs/PoseStamped  arrow from the sensor towards the
                                              sound: yaw = azimuth, positive left
  led/cmd          std_msgs/String            Kinect LED: led_sound while a sound is
                                              heard and for led_hold_s after it,
                                              then led_idle
"""

from collections import deque
import math

from audio_common_msgs.msg import AudioDataStamped, AudioInfo
from geometry_msgs.msg import PoseStamped
from kinect_v1_perception.doa import level_dbfs, NoiseGate, SrpPhat
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

# sample_format -> (numpy dtype, full scale)
FORMATS = {'S32LE': ('<i4', 2.0 ** 31), 'S16LE': ('<i2', 2.0 ** 15)}


class SoundLocalizer(Node):

    def __init__(self):
        super().__init__('sound_localizer')
        # Detection
        self._threshold_db = self.declare_parameter('threshold_db', 12.0).value
        self._min_level_db = self.declare_parameter('min_level_dbfs', -70.0).value
        self._noise_adapt_s = self.declare_parameter('noise_adapt_s', 2.0).value
        self._hold_s = self.declare_parameter('hold_s', 0.5).value
        # Direction
        # y of mic1..mic4 in the sensor frame [m] (y = left). Nominal Kinect v1
        # layout: mic1 alone on the sensor's right, the other three on its left
        # (side checked with claps on 2026-11-12, sensor upright).
        self._mic_y = self.declare_parameter(
            'mic_positions_y', [-0.113, 0.036, 0.076, 0.113]).value
        self._window = self.declare_parameter('window_samples', 1024).value
        self._speed_of_sound = self.declare_parameter('speed_of_sound', 343.0).value
        self._min_confidence = self.declare_parameter('min_confidence', 0.22).value
        self._onset_delay_s = self.declare_parameter('onset_delay_s', 0.016).value
        self._direction_period_s = self.declare_parameter('direction_period_s', 0.1).value
        self._frame_id = self.declare_parameter('frame_id', 'kinect_link').value
        # LED (the Kinect v1 has no blue: off, green, red, yellow and two blinks).
        # An empty led_sound leaves the LED alone.
        self._led_sound = self.declare_parameter('led_sound', 'yellow').value
        self._led_idle = self.declare_parameter('led_idle', 'green').value
        self._led_hold_s = self.declare_parameter('led_hold_s', 2.0).value

        # Built when audio/info and the first chunk tell us the format.
        self._info = None
        self._gate = None
        self._srp = None
        self._chunks = deque()   # the most recent audio, at least one window
        self._buffered = 0       # frames in _chunks

        self._index = 0          # chunk counter
        self._detected = False
        self._was_loud = False   # the previous chunk was loud
        self._last_loud = 0      # index of the last loud chunk
        self._next_estimate = 0  # index of the next direction estimate
        self._located = False    # the current sound already has a direction
        self._led_on = False     # the LED shows led_sound
        self._led_off_at = 0     # index at which the LED goes back to led_idle

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._detected_pub = self.create_publisher(Bool, 'sound/detected', latched)
        self._direction_pub = self.create_publisher(PoseStamped, 'sound/direction', 10)
        self._detected_pub.publish(Bool(data=False))
        self._led_pub = self.create_publisher(
            String, 'led/cmd', QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))

        self.create_subscription(AudioInfo, 'audio/info', self._on_info, latched)
        # Same QoS as the driver: reliable with a deep queue, a lost chunk
        # would break the time alignment inside the analysis window.
        self.create_subscription(
            AudioDataStamped, 'audio/data', self._on_audio,
            QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE))

    def _on_info(self, msg):
        if msg.sample_format not in FORMATS:
            self.get_logger().error(f'Unsupported sample_format {msg.sample_format}')
            return
        if msg.channels != len(self._mic_y):
            self.get_logger().error(
                f'{msg.channels} channels but {len(self._mic_y)} mic_positions_y')
            return
        self._info = msg
        self.get_logger().info(
            f'Listening: {msg.channels} ch {msg.sample_format} @ {msg.sample_rate} Hz')

    def _setup(self, frames):
        """Create the gate and the localizer; needs the chunk length."""
        chunk_s = frames / self._info.sample_rate
        self._gate = NoiseGate(chunk_s, self._threshold_db, self._min_level_db,
                               self._noise_adapt_s)
        self._srp = SrpPhat(self._mic_y, self._info.sample_rate, self._window,
                            self._speed_of_sound)
        self._hold_chunks = max(1, round(self._hold_s / chunk_s))
        self._onset_delay_chunks = max(1, round(self._onset_delay_s / chunk_s))
        self._period_chunks = max(1, round(self._direction_period_s / chunk_s))
        self._led_hold_chunks = round(self._led_hold_s / chunk_s)

    def _on_audio(self, msg):
        if self._info is None:
            return
        dtype, full_scale = FORMATS[self._info.sample_format]
        samples = np.frombuffer(msg.audio.data, dtype=dtype)
        samples = samples.reshape(-1, self._info.channels).astype(np.float32) / full_scale
        if self._gate is None:
            self._setup(len(samples))

        # Keep the last window of audio (a little more: whole chunks only).
        self._chunks.append(samples)
        self._buffered += len(samples)
        while self._buffered - len(self._chunks[0]) >= self._window:
            self._buffered -= len(self._chunks.popleft())

        self._index += 1
        level = level_dbfs(samples)
        loud = self._gate.update(level)

        if loud:
            self._last_loud = self._index
            if not self._was_loud:
                # Start of a sound (also a new one right after another, while
                # `detected` is still held). The first wavefront is the best
                # moment to measure the direction, before the echoes arrive:
                # wait just long enough for it to be inside the window.
                self._located = False
                self._next_estimate = self._index + self._onset_delay_chunks
            if not self._detected:
                self._set_detected(True)
        elif self._detected and self._index - self._last_loud > self._hold_chunks:
            self._set_detected(False)
        self._was_loud = loud

        # LED: back to idle led_hold_s after the sound has ended. The audio
        # stream never stops, so counting chunks is our clock.
        if self._led_on and not self._detected and self._index >= self._led_off_at:
            self._led_on = False
            self._led_pub.publish(String(data=self._led_idle))

        # One estimate per sound even if it was short (a clap); then, while it
        # continues, one every direction_period_s.
        if (self._detected and self._index >= self._next_estimate
                and (loud or not self._located)):
            self._located = True
            self._next_estimate = self._index + self._period_chunks
            self._locate(msg.header.stamp, level)

    def _set_detected(self, detected):
        self._detected = detected
        self._detected_pub.publish(Bool(data=detected))
        if not self._led_sound:
            return
        if detected and not self._led_on:
            self._led_on = True
            self._led_pub.publish(String(data=self._led_sound))
        elif not detected:
            self._led_off_at = self._index + self._led_hold_chunks

    def _locate(self, stamp, level):
        if self._buffered < self._window:
            return
        azimuth, confidence = self._srp.estimate(np.concatenate(self._chunks))
        side = 'left' if azimuth > 0 else 'right'
        text = (f'{math.degrees(azimuth):+4.0f} deg ({side}), confidence {confidence:.2f}, '
                f'level {level:.1f} dBFS (floor {self._gate.floor_db:.1f})')
        if confidence < self._min_confidence:
            self.get_logger().debug(f'sound, direction unclear: {text}')
            return
        self.get_logger().info(f'sound at {text}')

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self._frame_id
        # Rotation about z by the azimuth: the arrow (x axis) points at the sound.
        pose.pose.orientation.z = math.sin(azimuth / 2.0)
        pose.pose.orientation.w = math.cos(azimuth / 2.0)
        self._direction_pub.publish(pose)


def main():
    rclpy.init()
    node = SoundLocalizer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
