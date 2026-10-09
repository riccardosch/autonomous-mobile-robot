# Development log

Newest entries go at the top. Each entry records what was done, what was learned, the problems found and the next steps.
The *current* state of each part lives in the READMEs; this file keeps the history of how it got there.

---

## 2026-10-09: First LiDAR firmware, working on the hardware

### Done
- Created the PlatformIO project in `firmware_lidar/` (ESP32-C3, Arduino framework, native USB CDC) with three build environments: binary output, debug text, raw passthrough.
- Wrote the X2 packet parser (`x2_parser.*`): header sync, check bits, XOR checksum, angle interpolation across 0°, distance-based angle correction.
- Points are grouped into 360 one-degree bins per revolution (closest point wins) and sent as a binary frame with a CRC-16.
- Wrote `tools/read_scan.py` to read and validate the frames on the Pi.
- Tested the parser on the Pi (host build with g++) with synthetic packets: garbage before the header, wrap at 360°, corrupted checksum. CRC-16 in C matches Python's `binascii.crc_hqx`.
- Installed PlatformIO on the Pi (after `sudo apt install python3.12-venv`), flashed and tested the three modes on the hardware, in this order:
  - raw: about 7 KB/s, 274 `AA 55` headers in 3 s, 40 samples per packet;
  - debug: 8.0 scans/s (the LiDAR reports 7.9 Hz), about 320 of 360 bins with an echo, stable distances, 0 checksum errors;
  - binary: `read_scan.py` receives about 8 frames/s with no CRC errors and no lost frames.

- Added `tools/scan_viewer.py`, a live web viewer of the scans (the Pi is headless, the page is opened from the PC). Tested against the hardware: about 7.8 scans/s reach the page, with no CRC errors or lost frames.

### Learned
- Reading the kaiaai/LDS source: for the X2/X2L the library leaves M_SCTR **floating** (pin as input), it does not generate a PWM. The 8.3 scans/s of the first test are the LiDAR's default speed. Driving M_SCTR HIGH stops the motor. The firmware keeps the pin floating by default. Measured today with the same setup: 7.9-8.0 scans/s.
- In the ring start packet, `CT >> 1` is the scan frequency in 0.1 Hz units.

### Problems
- PlatformIO could not be installed at first: the system Python was missing the `venv` module. Fixed with `sudo apt install python3.12-venv`.
- **Upload failing** with "No serial data received" right after esptool loaded its stub. Lowering the upload speed did not help; uploading with `--no-stub` works and is now set in `platformio.ini`. Likely cause, not yet confirmed: GPIO20 is also UART0 RX, which the bootloader listens on, and the LiDAR keeps sending data into it.
- **Checksum errors when the Pi was not reading**: with the USB port closed, `Serial` writes blocked the loop for seconds and the LiDAR UART buffer overflowed (a scan rate of 0.33/s and a few checksum errors in the debug output). Fixed by writing to USB only when the whole frame fits in the buffer, and dropping it otherwise. Retested with the port closed for 8 s: 0 errors.

### Next steps
- Confirm the upload issue by flashing once with the LiDAR TX unplugged.
- micro-ROS on the ESP32 and the `/scan` topic (`sensor_msgs/LaserScan`).
- Dedicated 5V supply for the LiDAR.

---

## 2026-10-09: LiDAR identification, wiring and first test

### Done
- Salvaged the LiDAR module from a robot vacuum and identified it: label "YDS2MD-1", PCB "S-MAINBOARD-V2.1-TX_SEP", EAI logo.
- Worked out and verified the pinout of the 4-wire connector in the field (see [firmware_lidar/README.md](../firmware_lidar/README.md)).
- Wired the LiDAR to the ESP32-C3 SuperMini: TX → GPIO20, M_SCTR → GPIO3, common GND, 5V to the board's 5V pin.
- Ran a test with a throwaway sketch based on the [kaiaai/LDS](https://github.com/kaiaai/LDS) library (`LDS_YDLIDAR_X2_X2L`, 10 kHz motor PWM). Result: about 8.3 scans/s, with consistent distances and angles.
- Mechanics: designed the ESP32-C3 enclosure in OpenSCAD (USB-C slot, cable glands, 4 M3 tabs, snap-fit lid, PETG recommended) and generated the STLs. Designed the LiDAR bracket in Fusion, shaped to the actual profile of the enclosure.
- Set up the documentation structure (main README, firmware README, this log).

### Learned
- There is no public datasheet for the YDS2MD-1. The de facto reference is the YDLIDAR X2/X2L datasheet: the protocol is identical, as confirmed by real data.
- The LiDAR is single-channel: it only transmits, has no start command, and starts sending data as soon as it is powered.
- The head motor has its own 2-wire connector on the board. Power reaches the rotating head wirelessly (XKT-412 chip).
- Readings of 0.00 mm are points with no echo, not errors.

### Problems and mistakes
- **Wrong pinout at first**: the blue wire was initially taken for M_SCTR. The average voltage on an active UART TX line (about 1.7-1.8 V) looks like the default level of M_SCTR, so a multimeter is not enough. The TX line has to be identified from the data (`AA 55` header).
- **Power supply**: the ESP32 board's 5V pin is borderline for the LiDAR's inrush current (about 0.5-0.6 A). A dedicated 5V buck converter is needed.

### Next steps
- Install PlatformIO on the Raspberry Pi 5 and create the `firmware_lidar` project.
- First minimal firmware: motor PWM plus raw byte forwarding to USB, to check the whole chain.
- X2 protocol parser on the ESP32, sending each full scan as one binary frame.
- micro-ROS integration with the `/scan` topic.
- Check the LiDAR bracket: 0.2-0.3 mm diametral clearance, walls of at least 2 mm, mounting method. Measure the module with calipers, since it is an OEM part and the GrabCAD X2/G2 models are only indicative.
