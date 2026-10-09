# firmware_lidar

Firmware for the ESP32-C3 SuperMini. It reads the LiDAR packets over UART, controls the motor speed and sends the scans to the Raspberry Pi 5 over USB.

> **Status:** working on the hardware (2026-10-09): about 8 scans/s, ~320 of 360 bins with an echo, no checksum errors, binary frames read on the Pi without CRC errors.

## The LiDAR

| | |
|---|---|
| Label | YDS2MD-1 |
| PCB | S-MAINBOARD-V2.1-TX_SEP, EAI logo (Shenzhen EAI / YDLIDAR) |
| Family | OEM variant of YDLIDAR S2 / X2L |
| Datasheet | No public one. The YDLIDAR X2/X2L datasheet applies (identical protocol) |
| Power | 5 V, inrush current about 0.5-0.6 A |
| Interface | UART 115200 8N1, TX only |
| Startup | Transmits as soon as it is powered, no start command |
| Measured rate | About 8.3 scans/s with a 10 kHz PWM |

The motor is connected to a separate 2-wire connector on the LiDAR board. The rotating head is powered wirelessly (XKT-412 chip).

## 4-wire connector pinout (verified)

| Wire | Signal |
|---|---|
| Green | GND |
| Red | +5 V |
| Blue | TX (data) |
| Black | M_SCTR (motor speed control, PWM) |

> ⚠️ Do not identify the TX line with a multimeter: the average voltage on an active TX line (about 1.7-1.8 V) looks like the M_SCTR level. TX is recognised by its data: packets starting with `AA 55`.

## ESP32-C3 SuperMini connections

| LiDAR | ESP32-C3 | Notes |
|---|---|---|
| Blue (TX) | GPIO20 | RX of `Serial1` |
| Black (M_SCTR) | GPIO3 | Left floating by default, optional PWM through LEDC |
| Green (GND) | GND | Common ground |
| Red (+5 V) | 5V | Temporary: will move to a dedicated 5V buck |

Do not use GPIO 2, 8 and 9: they are strapping pins.

## Data protocol (YDLIDAR X2/X2L)

Every packet is little-endian:

| Field | Bytes | Description |
|---|---|---|
| PH | 2 | Header, `AA 55` on the wire (0x55AA) |
| CT | 1 | Packet type. Bit 0 = 1 marks the start of a new revolution |
| LSN | 1 | Number of samples in the packet |
| FSA | 2 | Angle of the first sample |
| LSA | 2 | Angle of the last sample |
| CS | 2 | Checksum |
| Si | 2 × LSN | Distance samples |

- **Distance:** `Distance_mm = Si / 4`. A value of 0 means no echo.
- **Raw angles:** `Angle_FSA = (FSA >> 1) / 64` degrees, and the same for LSA. Intermediate angles are linearly interpolated (watch out for the wrap at 360°).
- **Angle correction** (for non-zero distance):
  `AngCorrect = atan(21.8 * (155.3 - Distance) / (155.3 * Distance))`, in degrees, added to the raw angle.
- **Checksum:** XOR of the 16-bit words PH, FSA, every Si, `(LSN << 8) | CT` and LSA.

- **Scan frequency:** in the first packet of a revolution (CT bit 0 = 1), `CT >> 1` is the scan frequency in 0.1 Hz units.
- **Check bits:** bit 0 of FSA and of LSA is always 1. The parser uses it to reject false headers.

References: YDLIDAR X2/X2L datasheet, and the [kaiaai/LDS](https://github.com/kaiaai/LDS) driver (`LDS_YDLIDAR_X4.cpp`, based on YDLIDAR's own code), which uses the same formulas and constants.

## Motor speed (M_SCTR)

The kaiaai/LDS library used in the first test does **not** drive M_SCTR with a PWM signal for this model: it sets the pin as an input (floating), so the LiDAR spins at its default speed (about 8.3 scans/s). To stop the motor it drives the pin HIGH, so a higher PWM duty means a lower speed.

The firmware does the same by default (`MOTOR_PWM_DUTY_PERCENT = -1` in [src/config.h](src/config.h)). Setting a value from 0 to 100 enables a 10 kHz PWM with that duty.

## Code layout

| File | Contents |
|---|---|
| [platformio.ini](platformio.ini) | Board, framework and the three build environments |
| [src/config.h](src/config.h) | Pins, baud rate, motor settings, number of bins |
| [src/x2_parser.h](src/x2_parser.h), [src/x2_parser.cpp](src/x2_parser.cpp) | Byte-by-byte decoder of the X2 packets (header, checksum, angles, correction) |
| [src/main.cpp](src/main.cpp) | Setup, motor, grouping of the points into scans, output |
| [tools/read_scan.py](tools/read_scan.py) | Terminal reader for the binary frames, runs on the Pi |
| [tools/scan_viewer.py](tools/scan_viewer.py) | Live web viewer of the scans, runs on the Pi, opened from a browser |

## How a scan is built

1. Each byte from the LiDAR goes to `X2Parser::feed()`. Packets with a wrong checksum or invalid header are dropped and counted.
2. A packet with CT bit 0 = 1 marks the start of a new revolution: the previous scan is complete and is sent.
3. Each point goes into one of 360 bins (1° each, index = angle rounded to the nearest degree). If several points fall in the same bin the **closest** one is kept, which is the safe choice for obstacles. Bins with no echo stay at 0.

## Build and flash

The board is an ESP32-C3 SuperMini, built as `esp32-c3-devkitm-1` with the Arduino framework. It has no USB-UART bridge, so `Serial` uses the chip's native USB (`ARDUINO_USB_CDC_ON_BOOT=1`) and shows up on the Pi as `/dev/ttyACM0`.

| Environment | Output | Use |
|---|---|---|
| `esp32c3` (default) | Binary frames | Normal use, read by `tools/read_scan.py` |
| `esp32c3_debug` | Text statistics once per second | Checking the parser with `pio device monitor` |
| `esp32c3_raw` | Raw LiDAR bytes | Checking the wiring (look for `AA 55`) |

PlatformIO is installed in `~/.platformio/penv/bin` (not in `PATH` by default):

```bash
cd firmware_lidar
~/.platformio/penv/bin/pio run -e esp32c3_debug -t upload   # build and flash
~/.platformio/penv/bin/pio device monitor                   # Ctrl+C to exit
```

**Uploading with the LiDAR connected.** With the default settings esptool connects, loads its stub and then stops answering ("No serial data received"). Uploading with the ROM loader only (`--no-stub`) works, so `platformio.ini` sets `upload_flags = --no-stub` and `upload_speed = 115200`. The likely cause is that GPIO20 is also the RX of UART0, which the bootloader listens on, and the LiDAR keeps sending data into it. This has not been confirmed (e.g. by flashing with the LiDAR unplugged).

If the upload does not start at all, hold BOOT, press and release RESET, then release BOOT, and retry.

**USB writes never block.** If the Pi is not reading the port, a USB write can block for seconds, and meanwhile the LiDAR UART buffer overflows. The firmware writes a frame only if it fits entirely in the USB buffer, and drops it otherwise (`usb dropped` in the debug output).

## Output format to the Pi

### Binary frame (`esp32c3`)

One frame per revolution, little-endian:

| Offset | Size | Field |
|---|---|---|
| 0 | 2 | Sync `A5 5A` |
| 2 | 1 | Frame version (1) |
| 3 | 2 | Sequence number (wraps at 65535) |
| 5 | 4 | Duration of the revolution in µs |
| 9 | 2 | Number of bins N (360) |
| 11 | 2·N | Distances in mm, index = angle in degrees, 0 = no echo |
| 11+2·N | 2 | CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over bytes 2 … 10+2·N |

With N = 360 a frame is 733 bytes, about 6 KB/s at 8 scans/s.

```bash
python3 tools/read_scan.py --port /dev/ttyACM0          # one line per scan
python3 tools/read_scan.py --port /dev/ttyACM0 --dump   # also every distance
```

### Live viewer

`scan_viewer.py` reads the frames and serves a web page that draws each scan in real time (polar plot, range rings, nearest obstacle, scan rate, lost frames, CRC errors, zoom, pause, angle and distance under the mouse). It only needs Python's standard library and pyserial.

The Pi has no screen, so the page is opened from the PC's browser:

```bash
python3 tools/scan_viewer.py                 # listens on 127.0.0.1:8000
```

- **Through VS Code (Remote-SSH):** the Ports tab forwards port 8000; open `http://localhost:8000` on the PC.
- **Through SSH:** `ssh -L 8000:localhost:8000 riccardo@<pi-address>`, then open `http://localhost:8000`.
- **Directly on the LAN:** start it with `--host 0.0.0.0` and open `http://<pi-address>:8000`. Anyone on the network can then see the page.

Only one program at a time can open the serial port: stop `read_scan.py` or `pio device monitor` first.

The plot assumes 0° points forward (up) and angles grow clockwise, as in the YDLIDAR convention. This still has to be checked on the robot, e.g. by placing an object in front of the LiDAR.

### Debug text (`esp32c3_debug`)

```
scans/s 8.00 (lidar 7.9 Hz) | packets ok 1299, checksum err 0, framing err 0 | valid bins 319/360 | 0:381 90:132 180:644 270:140 mm | usb dropped 0
```

### Next step

Phase 2 replaces the binary frame with micro-ROS publishing the `/scan` topic (`sensor_msgs/LaserScan`). The parser and the scan building stay the same.
