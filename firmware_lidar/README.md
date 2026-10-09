# firmware_lidar

Firmware for the ESP32-C3 SuperMini. It reads the LiDAR packets over UART, controls the motor speed and sends the scans to the Raspberry Pi 5 over USB.

> **Status:** not written yet. For now this document collects the verified hardware information and the protocol. The build and output format sections will be filled in along with the firmware.

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
| Black (M_SCTR) | GPIO3 | PWM through LEDC |
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

Reference: YDLIDAR X2/X2L datasheet and development manual (YDLIDAR website). The formulas still have to be checked against the official document when the parser is implemented.

## Build and flash

*To be completed.* The project will use PlatformIO (board `esp32-c3-devkitm-1`, Arduino framework, USB CDC on boot).

## Output format to the Pi

*To be defined.* Phase 1: one binary frame per scan (360 distances in mm plus a CRC). Phase 2: micro-ROS with the `/scan` topic (`sensor_msgs/LaserScan`).
