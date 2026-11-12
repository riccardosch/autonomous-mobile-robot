# kinect_v1_perception

Perception nodes on top of the [Kinect v1 driver](https://github.com/riccardosch/kinect_v1_ros2). The driver publishes images and raw audio; these nodes turn them into "there is a noise on the left", "it is speech" and "there is a person in the image".

| Node | Does | Needs |
|---|---|---|
| `sound_localizer` | Detects sounds, estimates their direction, turns the Kinect LED yellow | driver `aux` component |
| `sound_classifier` | Names the kind of sound with YAMNet (521 AudioSet classes) | `sound_localizer`, `~/agv_venv` |
| `yolo_detector` | Finds objects and people in the RGB image with YOLO26n | driver `camera` component, `~/agv_venv` |

All topics are relative: the nodes run in the driver's namespace (`/kinect`).

## Setup (once)

The driver lives in its own workspace (`~/kinect`, see its README). The two neural networks need Python packages that are not in Ubuntu, so they go in a virtual environment that still sees the system (ROS) packages:

```bash
sudo apt install ros-jazzy-vision-msgs
python3 -m venv --system-site-packages ~/agv_venv
~/agv_venv/bin/pip install "numpy<2" ai-edge-litert ultralytics
~/agv_venv/bin/pip uninstall -y setuptools     # see the note below
```

- **`numpy<2`**: ROS Jazzy, SciPy and cv_bridge on Ubuntu 24.04 are built for NumPy 1.26. pip pulls NumPy 2 unless told otherwise.
- **setuptools**: pip installs setuptools ≥ 80 in the environment, which breaks `colcon build --symlink-install`. Removing it makes the environment use the system one (68). Ultralytics may reinstall it during an export: remove it again.

Models (not in git), in `~/.cache/agv/`:

```bash
mkdir -p ~/.cache/agv && cd ~/.cache/agv
curl -O https://storage.googleapis.com/mediapipe-models/audio_classifier/yamnet/float32/1/yamnet.tflite
~/agv_venv/bin/python -c "from ultralytics import YOLO; YOLO('yolo26n.pt').export(format='ncnn', imgsz=640)"
~/agv_venv/bin/pip uninstall -y setuptools
```

The second command downloads `yolo26n.pt` and converts it to `yolo26n_ncnn_model/`.

## Build

Build **from inside the environment**, so the nodes start with its Python:

```bash
cd raspberry
source /opt/ros/jazzy/setup.bash && source ~/kinect/install/setup.bash
source ~/agv_venv/bin/activate
python -m colcon build --symlink-install --packages-select kinect_v1_perception
```

A plain `colcon build` makes the nodes use `/usr/bin/python3`, and `sound_classifier` and `yolo_detector` then fail to import their libraries.

## Run

```bash
source ~/kinect/install/setup.bash
source ~/agv/autonomous-mobile-robot/raspberry/install/setup.bash
ros2 launch kinect_v1_perception perception.launch.py
```

| Argument | Default | |
|---|---|---|
| `driver` | `true` | Also start the Kinect driver (`false` if it is already running) |
| `sound` | `true` | `sound_localizer` |
| `sound_type` | `true` | `sound_classifier` (needs `sound:=true`) |
| `yolo` | `true` | `yolo_detector` |
| `depth_mode` | `mm` | Passed to the driver: `mm`, `registered`, `off` |
| `namespace` | `kinect` | |

One node alone: `ros2 run kinect_v1_perception yolo_detector --ros-args -r __ns:=/kinect`.

Only one process can own the Kinect: stop a running driver before launching with `driver:=true`.

## Topics

| Topic | Type | From |
|---|---|---|
| `sound/detected` | `std_msgs/Bool`, latched, published on change | `sound_localizer` |
| `sound/direction` | `geometry_msgs/PoseStamped` in `kinect_link`: yaw = azimuth, positive to the left | `sound_localizer` |
| `led/cmd` | `std_msgs/String`: `yellow` on a sound, `green` 2 s after it ends | `sound_localizer` |
| `sound/type` | `std_msgs/String`: AudioSet class name (`Speech`, `Clapping`, `Doorbell`, ...) | `sound_classifier` |
| `detections` | `vision_msgs/Detection2DArray`: class name, score, box centre and size in pixels; header of the source image | `yolo_detector` |
| `detections/image` | `sensor_msgs/Image` `rgb8` with the boxes drawn, best effort, only while subscribed | `yolo_detector` |

To watch the boxes from a PC: RViz2 → Add → By topic → `/kinect/detections/image` → Image, with *Reliability Policy* = *Best Effort*.

## How it works

**`sound_localizer`** ([doa.py](kinect_v1_perception/doa.py)). Every 16 ms chunk gets a level in dBFS. A chunk is a sound when it is 12 dB above the noise floor, which is a slow moving average of the level (a constant noise becomes the new floor after a while). At the start of a sound, the direction is estimated with SRP-PHAT over the 6 microphone pairs on the last 64 ms; the estimate is published only if its confidence is at least 0.22. The Kinect LED has no blue: the sound colour is yellow.

**`sound_classifier`** ([yamnet.py](kinect_v1_perception/yamnet.py)). Half a second after `sound/detected` becomes true, the last 0.975 s of audio (mean of the 4 channels) goes through YAMNet; the best class with a score of at least 0.3 is published. Repeats every 0.5 s while the sound lasts.

**`yolo_detector`**. Each RGB image goes through YOLO26n (NCNN). The subscription keeps only the newest image, so the frames that arrive while the network is busy are skipped.

## Parameters

| Node | Name | Default | |
|---|---|---|---|
| `sound_localizer` | `threshold_db` | `12.0` | Level above the noise floor that counts as a sound |
| | `min_level_dbfs` | `-70.0` | Never a sound below this level |
| | `noise_adapt_s` | `2.0` | Time constant of the noise floor |
| | `hold_s` | `0.5` | `sound/detected` stays true this long after the last loud chunk |
| | `mic_positions_y` | `[-0.113, 0.036, 0.076, 0.113]` | y of mic1..mic4 in `kinect_link` [m]; nominal layout |
| | `min_confidence` | `0.22` | Minimum SRP-PHAT confidence to publish a direction |
| | `onset_delay_s` | `0.016` | Delay between the start of a sound and the first estimate |
| | `led_sound`, `led_idle`, `led_hold_s` | `yellow`, `green`, `2.0` | LED colours and hold time; empty `led_sound` leaves the LED alone |
| `sound_classifier` | `model_path` | `~/.cache/agv/yamnet.tflite` | |
| | `min_score` | `0.3` | Minimum score to publish a class |
| | `ignore_labels` | `[Silence, "Inside, small room"]` | Classes never published |
| | `delay_s`, `period_s` | `0.5`, `0.5` | First classification after the onset, then period |
| `yolo_detector` | `model` | `~/.cache/agv/yolo26n_ncnn_model` | Exported folder or `.pt` file |
| | `min_score` | `0.4` | Minimum score of a box |
| | `image_size` | `640` | Network input size |

## Tests

Unit tests (no hardware): `python -m pytest test` from the package folder, inside the environment → 11 passed. With the system Python the YAMNet test is skipped.

Hardware tests, Raspberry Pi 5, 2026-11-12:

| # | Test | Result |
|---|---|---|
| 1 | Quiet room, 12 s | No detection |
| 2 | Direction: 3 claps each at about 45° left, centre, 45° right, 1–2 m away (recording replayed through the node) | +48°, +48°, +48° / −2°, +1°, +1° / −32°, −35°, −35°; confidence 0.25–0.41. Confirmed live by the user |
| 3 | LED | Yellow at the first clap, green 2 s after the last sound. Confirmed live by the user |
| 4 | Sound type: claps on the same recording | `Hands`, `Clapping`, `Finger snapping`; `Silence` on quiet parts; 3 ms per inference. Claps, speech, knocking and phone sounds confirmed live by the user (no output recorded) |
| 5 | YOLO26n speed, 640×480 image | PyTorch 323 ms, NCNN 106 ms per image |
| 6 | YOLO live, alone | 7.3 images/s, `person` detected |
| 7 | Everything together (`perception.launch.py`) | All nodes up, YOLO 6.0 images/s (161 ms per image) |

Findings from test 2:
- mic1 is the single microphone on the sensor's **right**, with the sensor upright.
- The estimate must be made right at the onset (16 ms): later the room echoes spoil it.
- On the right side the angle is underestimated (−35° for about −45°) with lower confidence; not explained yet.

## Limits and to do

- A linear array cannot tell front from back: the azimuth is in [−90°, +90°].
- Thresholds were tuned on one recording in one room with the robot still; to be checked with the motors running.
- Sound type: compare other models against YAMNet and keep the best one.
- YOLO26n knows the 80 COCO classes only.
