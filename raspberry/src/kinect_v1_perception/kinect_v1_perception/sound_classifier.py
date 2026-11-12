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
Say what kind of sound the Kinect microphones are hearing.

Works after sound_localizer: when it reports a sound (sound/detected), this
node waits until the sound is inside the last 0.975 s of audio, asks YAMNet
what it is, and repeats while the sound continues.

Subscribes (relative names, run the node in the driver's namespace):
  audio/info      audio_common_msgs/AudioInfo         format of the stream (latched)
  audio/data      audio_common_msgs/AudioDataStamped  interleaved PCM chunks
  sound/detected  std_msgs/Bool                       from sound_localizer

Publishes:
  sound/type      std_msgs/String   AudioSet class name, e.g. 'Speech', 'Clapping'
"""

from collections import deque
import os

from audio_common_msgs.msg import AudioDataStamped, AudioInfo
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

from kinect_v1_perception.sound_localizer import FORMATS
from kinect_v1_perception.yamnet import Yamnet


class SoundClassifier(Node):

    def __init__(self):
        super().__init__('sound_classifier')
        model_path = os.path.expanduser(
            self.declare_parameter('model_path', '~/.cache/agv/yamnet.tflite').value)
        self._min_score = self.declare_parameter('min_score', 0.3).value
        # Labels that say nothing about the event.
        self._ignore = set(self.declare_parameter(
            'ignore_labels', ['Silence', 'Inside, small room']).value)
        self._delay_s = self.declare_parameter('delay_s', 0.5).value
        self._period_s = self.declare_parameter('period_s', 0.5).value

        self._model = Yamnet(model_path)
        self._info = None
        self._chunks = deque()   # the most recent mono audio, at least one model window
        self._buffered = 0
        self._index = 0          # chunk counter: our clock
        self._detected = False
        self._due = None         # index of the next classification, None = nothing to do

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._type_pub = self.create_publisher(String, 'sound/type', 10)
        self.create_subscription(AudioInfo, 'audio/info', self._on_info, latched)
        self.create_subscription(
            AudioDataStamped, 'audio/data', self._on_audio,
            QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE))
        self.create_subscription(Bool, 'sound/detected', self._on_detected, latched)

    def _on_info(self, msg):
        if msg.sample_format not in FORMATS or msg.sample_rate != Yamnet.SAMPLE_RATE:
            self.get_logger().error(
                f'Need {Yamnet.SAMPLE_RATE} Hz S32LE/S16LE audio, got '
                f'{msg.sample_rate} Hz {msg.sample_format}')
            return
        self._info = msg
        self.get_logger().info(f'Ready: YAMNet, {len(self._model.labels)} kinds of sound')

    def _on_detected(self, msg):
        if msg.data and not self._detected and self._due is None:
            # A sound has just started: classify when it is well inside the window.
            self._due = self._index + self._chunks_in(self._delay_s)
        self._detected = msg.data

    def _chunks_in(self, seconds):
        frames = len(self._chunks[-1]) if self._chunks else 256
        return max(1, round(seconds * Yamnet.SAMPLE_RATE / frames))

    def _on_audio(self, msg):
        if self._info is None:
            return
        dtype, full_scale = FORMATS[self._info.sample_format]
        samples = np.frombuffer(msg.audio.data, dtype=dtype).reshape(-1, self._info.channels)
        mono = samples.mean(axis=1).astype(np.float32) / full_scale

        self._chunks.append(mono)
        self._buffered += len(mono)
        while self._buffered - len(self._chunks[0]) >= self._model.window:
            self._buffered -= len(self._chunks.popleft())
        self._index += 1

        if self._due is not None and self._index >= self._due:
            # Again later if the sound is still going on, otherwise done.
            self._due = self._index + self._chunks_in(self._period_s) if self._detected else None
            self._classify()

    def _classify(self):
        if self._buffered < self._model.window:
            return
        window = np.concatenate(self._chunks)[-self._model.window:]
        results = [(label, score) for label, score in self._model.classify(window, top=5)
                   if label not in self._ignore]
        text = ', '.join(f'{label} {score:.2f}' for label, score in results[:3])
        if not results or results[0][1] < self._min_score:
            self.get_logger().debug(f'sound type unclear: {text}')
            return
        self.get_logger().info(f'sound type: {text}')
        self._type_pub.publish(String(data=results[0][0]))


def main():
    rclpy.init()
    try:
        node = SoundClassifier()
    except (ImportError, OSError) as error:
        raise SystemExit(f'sound_classifier cannot start: {error}\n'
                         'See the yamnet.py header for what to install.')
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
