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

"""Tests of the array signal processing on synthetic audio (no hardware)."""

import math

from kinect_v1_perception.doa import level_dbfs, NoiseGate, SrpPhat
import numpy as np
import pytest

MIC_Y = [-0.113, 0.036, 0.076, 0.113]
RATE = 16000
WINDOW = 1024
SPEED = 343.0


def simulate(azimuth_deg, noise=0.05, seed=0):
    """White-noise source at `azimuth_deg`, far away, as heard by each microphone."""
    rng = np.random.default_rng(seed)
    n = 4 * WINDOW
    source = np.fft.rfft(rng.standard_normal(n))
    freqs = np.fft.rfftfreq(n, 1.0 / RATE)
    channels = []
    for y in MIC_Y:
        arrival_s = -y * math.sin(math.radians(azimuth_deg)) / SPEED
        delayed = np.fft.irfft(source * np.exp(-2j * np.pi * freqs * arrival_s), n)
        channels.append(delayed + noise * rng.standard_normal(n))
    x = np.stack(channels, axis=1)
    return x[n // 2:n // 2 + WINDOW]


@pytest.mark.parametrize('azimuth_deg', [-70, -40, -15, 0, 10, 30, 60])
def test_srp_phat_finds_the_source(azimuth_deg):
    srp = SrpPhat(MIC_Y, RATE, WINDOW, SPEED)
    azimuth, confidence = srp.estimate(simulate(azimuth_deg))
    assert math.degrees(azimuth) == pytest.approx(azimuth_deg, abs=3.0)
    assert confidence > 0.5


def test_srp_phat_low_confidence_without_a_source():
    srp = SrpPhat(MIC_Y, RATE, WINDOW, SPEED)
    rng = np.random.default_rng(1)
    _, confidence = srp.estimate(rng.standard_normal((WINDOW, 4)))
    assert confidence < 0.2


def test_level_dbfs():
    t = np.arange(1600) / RATE
    sine = 0.1 * np.sin(2 * np.pi * 1000 * t)[:, None]
    assert level_dbfs(sine) == pytest.approx(20 * math.log10(0.1 / math.sqrt(2)), abs=0.1)
    # A constant offset is not sound.
    assert level_dbfs(sine + 0.5) == pytest.approx(level_dbfs(sine), abs=0.1)


def test_noise_gate():
    gate = NoiseGate(chunk_s=0.016, threshold_db=12.0)
    assert not gate.update(-240.0)                          # zeros at stream start: ignored
    assert not any(gate.update(-60.0) for _ in range(100))  # steady background
    assert not gate.update(-55.0)                           # small change
    assert gate.update(-30.0)                               # a clap
    assert gate.floor_db == pytest.approx(-60.0, abs=1.0)   # the clap did not move the floor
    # A constant new noise becomes the background after a while.
    results = [gate.update(-30.0) for _ in range(5000)]
    assert results[0] and not results[-1]
