# Development log

Newest entries go at the top. Each entry records what was done, what was learned, the problems found and the next steps.
The *current* state of each part lives in the READMEs; this file keeps the history of how it got there.

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
