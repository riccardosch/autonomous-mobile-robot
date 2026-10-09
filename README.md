# Autonomous Mobile Robot (AGV)

An autonomous mobile robot built on a Raspberry Pi 5 and ROS 2. It uses a LiDAR salvaged from a robot vacuum cleaner for mapping (SLAM) and navigation.

## Architecture

```
LiDAR YDS2MD-1 ──UART 115200──▶ ESP32-C3 SuperMini ──USB-C──▶ Raspberry Pi 5
  (TX data)                       - packet parsing              - micro-ROS agent
  (M_SCTR ◀── PWM)                - motor control               - /scan topic
                                  - scan forwarding             - SLAM + wheel odometry
```

## Hardware

| Component | Role | Notes |
|---|---|---|
| Raspberry Pi 5 | Onboard computer, ROS 2 | |
| ESP32-C3 SuperMini | LiDAR ↔ Pi interface | Connected over USB-C |
| LiDAR YDS2MD-1 (EAI) | 360° scanning | OEM variant of the YDLIDAR X2/X2L family. See [firmware_lidar/README.md](firmware_lidar/README.md) |

## Repository layout

| Folder / file | Contents |
|---|---|
| [firmware_lidar/](firmware_lidar/) | PlatformIO firmware for the ESP32-C3 that reads the LiDAR |
| [docs/devlog.md](docs/devlog.md) | Development log (chronological) |

## Project status

| Part | Status |
|---|---|
| LiDAR identification and pinout | ✅ Done and verified |
| LiDAR ↔ ESP32-C3 wiring | ✅ Done |
| LiDAR test (throwaway sketch using the kaiaai/LDS library) | ✅ About 8.3 scans/s, consistent data |
| ESP32-C3 firmware (parser + USB output) | ✅ Working on the hardware: ~8 scans/s, no checksum or CRC errors |
| micro-ROS and `/scan` topic | ⏳ To do |
| Wheel odometry | ⏳ To do |
| SLAM | ⏳ To do |
| ESP32-C3 enclosure (OpenSCAD) | ✅ STLs generated |
| LiDAR bracket (Fusion) | 🔍 To check: tolerances and mounting |

What was done and when is recorded in the [development log](docs/devlog.md).

## License

Apache 2.0, see [LICENSE](LICENSE).
