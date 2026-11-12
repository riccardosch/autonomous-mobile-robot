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
YAMNet sound classifier (no ROS in this file).

YAMNet is a small neural network from Google trained on AudioSet: it listens
to 0.975 s of mono audio at 16 kHz and gives a score in [0, 1] to each of 521
kinds of sound ('Speech', 'Doorbell', 'Glass', 'Dog', 'Clapping', ...).

Needs the LiteRT runtime (pip package ai-edge-litert) and the model file:
  curl -o ~/.cache/agv/yamnet.tflite https://storage.googleapis.com/\
mediapipe-models/audio_classifier/yamnet/float32/1/yamnet.tflite
"""

import zipfile

from ai_edge_litert.interpreter import Interpreter
import numpy as np


class Yamnet:
    """Load the model once, then classify windows of audio."""

    SAMPLE_RATE = 16000

    def __init__(self, model_path):
        # The .tflite file is also a zip archive that carries the class names.
        with zipfile.ZipFile(model_path) as archive:
            self.labels = archive.read('yamnet_label_list.txt').decode().splitlines()
        self._interpreter = Interpreter(model_path=model_path)
        self._interpreter.allocate_tensors()
        self._input = self._interpreter.get_input_details()[0]
        self._output = self._interpreter.get_output_details()[0]
        self.window = int(self._input['shape'][0])  # 15600 samples = 0.975 s

    def classify(self, waveform, top=3):
        """
        Return the `top` most likely sounds as [(label, score), ...].

        `waveform` holds `window` mono samples in [-1, 1].
        """
        x = np.asarray(waveform, dtype=np.float32)
        x = x - x.mean()
        self._interpreter.set_tensor(self._input['index'], x.reshape(self._input['shape']))
        self._interpreter.invoke()
        scores = self._interpreter.get_tensor(self._output['index']).ravel()
        best = np.argsort(scores)[::-1][:top]
        return [(self.labels[i], float(scores[i])) for i in best]
