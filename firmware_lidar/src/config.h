#pragma once

// ---- Pins (ESP32-C3 SuperMini) ----
// Do not use GPIO 2, 8 and 9: they are strapping pins.
constexpr int PIN_LIDAR_RX = 20;     // LiDAR TX (blue wire)
constexpr int PIN_LIDAR_MOTOR = 3;   // LiDAR M_SCTR (black wire)

// ---- LiDAR UART ----
constexpr unsigned long LIDAR_BAUD = 115200;
constexpr size_t LIDAR_RX_BUFFER = 2048;  // ~180 ms of data at 115200 baud

// ---- Motor speed control (M_SCTR) ----
// A negative duty leaves M_SCTR floating (pin as input): the LiDAR then spins at
// its own default speed. This is the configuration verified on 2026-10-09
// (~8.3 scans/s). Driving M_SCTR HIGH slows the motor down to a stop, so a
// higher duty means a lower speed.
constexpr int MOTOR_PWM_DUTY_PERCENT = -1;
constexpr uint32_t MOTOR_PWM_FREQ_HZ = 10000;
constexpr uint8_t MOTOR_PWM_RESOLUTION_BITS = 8;
constexpr uint8_t MOTOR_PWM_CHANNEL = 0;  // only used by arduino-esp32 2.x

// ---- USB output ----
// Larger than one binary frame (733 bytes), so a whole frame fits at once.
constexpr size_t USB_TX_BUFFER = 2048;

// ---- Scan output ----
constexpr int SCAN_BINS = 360;  // 1 degree per bin, index = angle in degrees

// ---- Debug output ----
constexpr unsigned long DEBUG_PERIOD_MS = 1000;
