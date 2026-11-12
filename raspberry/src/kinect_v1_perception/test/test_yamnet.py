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

"""Tests of the YAMNet wrapper. Skipped without the runtime or the model file."""

import os

import numpy as np
import pytest

MODEL = os.path.expanduser('~/.cache/agv/yamnet.tflite')


def test_silence_and_noise():
    pytest.importorskip('ai_edge_litert')
    if not os.path.exists(MODEL):
        pytest.skip('model not downloaded')
    from kinect_v1_perception.yamnet import Yamnet
    model = Yamnet(MODEL)
    assert len(model.labels) == 521
    assert model.classify(np.zeros(model.window))[0][0] == 'Silence'
    noise = 0.3 * np.random.default_rng(0).standard_normal(model.window)
    results = model.classify(noise, top=3)
    assert len(results) == 3 and results[0][0] != 'Silence'
    assert results[0][1] >= results[1][1] >= results[2][1]
