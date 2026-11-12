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
Signal processing for the Kinect microphone array (no ROS in this file).

- level_dbfs: loudness of a chunk of audio.
- NoiseGate: decides whether a chunk is "a sound" by comparing its level
  with a slowly adapting estimate of the background noise.
- SrpPhat: direction of arrival (azimuth) of a sound on a linear array.

Conventions (REP 103): x forward, y left. The microphones lie on the y axis.
The azimuth is the angle of the source from the x axis, positive to the left.
A linear array cannot tell front from back: the result is in [-90, +90] deg.
"""

import itertools
import math

import numpy as np

# Below this level a chunk is digital silence, not a measurement of the room.
SILENCE_DB = -120.0


def level_dbfs(samples):
    """
    Return the RMS level of `samples` in dB relative to full scale.

    `samples` has shape (frames, channels) with values in [-1, 1]. The mean of
    each channel is removed first, so a constant offset does not count as sound.
    """
    x = samples - samples.mean(axis=0)
    rms = math.sqrt(float(np.mean(x * x)))
    return 20.0 * math.log10(rms + 1e-12)


class NoiseGate:
    """
    Tell sounds from background noise.

    The noise floor is an exponential moving average of the level (in dB).
    A chunk is loud when it exceeds the floor by `threshold_db` and is above
    `min_level_db`. While a chunk is loud the floor still adapts, but
    `loud_slowdown` times slower: a short sound barely moves it, a constant
    new noise (a fan, the robot's motors) becomes the new floor after a while.
    """

    def __init__(self, chunk_s, threshold_db=12.0, min_level_db=-70.0,
                 adapt_s=2.0, loud_slowdown=20.0):
        self.threshold_db = threshold_db
        self.min_level_db = min_level_db
        self.floor_db = None
        self._alpha = min(1.0, chunk_s / adapt_s)
        self._alpha_loud = self._alpha / loud_slowdown

    def update(self, level_db):
        """Feed the level of one chunk; return True if the chunk is loud."""
        if level_db < SILENCE_DB:
            return False  # all zeros (the stream starts like this): not a noise level
        if self.floor_db is None:
            self.floor_db = level_db
            return False
        loud = level_db > max(self.floor_db + self.threshold_db, self.min_level_db)
        alpha = self._alpha_loud if loud else self._alpha
        self.floor_db += alpha * (level_db - self.floor_db)
        return loud


class SrpPhat:
    """
    Azimuth of a far sound source with SRP-PHAT on a linear array.

    A sound from the left reaches the left microphones first. For each pair
    of microphones (i, j) the GCC-PHAT cross-correlation has a peak at the
    lag t_i - t_j between the two arrival times. For a source at azimuth a:

        t_i - t_j = -(y_i - y_j) * sin(a) / c

    For every candidate azimuth we read each pair's correlation at the lag
    that azimuth predicts and add them up (the "steered response power").
    The azimuth with the highest sum wins.

    PHAT keeps only the phase of each frequency (the magnitude is set to 1):
    the correlation peak gets sharp and echoes disturb it less.
    """

    def __init__(self, mic_y, sample_rate, window, speed_of_sound=343.0,
                 interp=8, f_min=300.0, f_max=6000.0, angle_step_deg=1.0):
        mic_y = np.asarray(mic_y, dtype=np.float64)
        self.window = int(window)
        self._n_fft = 2 * self.window
        # Zero-padding the spectrum before the inverse FFT interpolates the
        # correlation: lags are in steps of 1 / (sample_rate * interp).
        self._n_lags = self._n_fft * interp
        self._taper = np.hanning(self.window)[:, None]

        freqs = np.fft.rfftfreq(self._n_fft, 1.0 / sample_rate)
        self._band = (freqs >= f_min) & (freqs <= f_max)

        self.angles = np.radians(np.arange(-90.0, 90.0 + angle_step_deg, angle_step_deg))
        self._pairs = list(itertools.combinations(range(len(mic_y)), 2))
        # _lag_index[p][a]: where to read pair p's correlation for angle a.
        # Negative lags wrap around to the end of the array.
        self._lag_index = []
        for i, j in self._pairs:
            delay_s = -(mic_y[i] - mic_y[j]) * np.sin(self.angles) / speed_of_sound
            lags = np.rint(delay_s * sample_rate * interp).astype(int)
            self._lag_index.append(lags % self._n_lags)

        # Sum reached when all pairs are perfectly aligned: used to scale the
        # confidence to [0, 1].
        peak = 2.0 * np.count_nonzero(self._band) / self._n_lags
        self._ideal = peak * len(self._pairs)

    def estimate(self, samples):
        """
        Return (azimuth [rad], confidence in [0, 1]).

        `samples` has shape (window, channels). The confidence is the height
        of the winning peak: about 1 for a clean single source, near 0 for
        noise with no direction.
        """
        x = samples[-self.window:].astype(np.float64)
        x = (x - x.mean(axis=0)) * self._taper
        spectrum = np.fft.rfft(x, self._n_fft, axis=0)

        power = np.zeros(len(self.angles))
        for (i, j), lag_index in zip(self._pairs, self._lag_index):
            cross = spectrum[:, i] * np.conj(spectrum[:, j])
            cross /= np.abs(cross) + 1e-12  # PHAT: keep the phase only
            cross[~self._band] = 0.0
            correlation = np.fft.irfft(cross, self._n_lags)
            power += correlation[lag_index]

        best = int(np.argmax(power))
        confidence = float(np.clip(power[best] / self._ideal, 0.0, 1.0))
        return float(self.angles[best]), confidence
