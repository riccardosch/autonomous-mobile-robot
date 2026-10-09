// LiDAR firmware for the ESP32-C3 SuperMini.
//
// Reads the YDS2MD-1 LiDAR (YDLIDAR X2/X2L protocol) on Serial1, groups the
// points of each revolution into 1-degree bins and sends one scan at a time to
// the Raspberry Pi over the native USB serial port. The output format is chosen
// at build time, see platformio.ini and README.md.

#include <Arduino.h>

#include "config.h"
#include "x2_parser.h"

#if !defined(LIDAR_OUTPUT_BINARY) && !defined(LIDAR_OUTPUT_DEBUG) && !defined(LIDAR_OUTPUT_RAW)
#error "Select an output mode: LIDAR_OUTPUT_BINARY, LIDAR_OUTPUT_DEBUG or LIDAR_OUTPUT_RAW"
#endif

namespace {

X2Parser parser;

// Scan being filled and the last completed one. Distances in mm, 0 = no echo.
uint16_t building[SCAN_BINS];
uint16_t completed[SCAN_BINS];
uint16_t scan_seq = 0;
uint32_t scan_duration_us = 0;
uint32_t ring_start_us = 0;
bool have_ring_start = false;

// Statistics for the debug output
uint32_t scans_total = 0;
uint32_t usb_dropped = 0;
float lidar_reported_freq_hz = 0;

// Writes to USB only if the whole buffer fits, otherwise drops it. Without this
// check a write blocks for seconds when the Pi is not reading, and meanwhile the
// LiDAR UART buffer overflows (seen as checksum errors on 2026-10-09).
bool usbWrite(const uint8_t* data, size_t len) {
  if (Serial.availableForWrite() < int(len)) {
    usb_dropped++;
    return false;
  }
  Serial.write(data, len);
  return true;
}

void setupMotor() {
  if (MOTOR_PWM_DUTY_PERCENT < 0) {
    pinMode(PIN_LIDAR_MOTOR, INPUT);  // floating: LiDAR default speed
    return;
  }
  const uint32_t max_duty = (1u << MOTOR_PWM_RESOLUTION_BITS) - 1;
  const uint32_t duty = max_duty * MOTOR_PWM_DUTY_PERCENT / 100;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttach(PIN_LIDAR_MOTOR, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_RESOLUTION_BITS);
  ledcWrite(PIN_LIDAR_MOTOR, duty);
#else
  ledcSetup(MOTOR_PWM_CHANNEL, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_RESOLUTION_BITS);
  ledcAttachPin(PIN_LIDAR_MOTOR, MOTOR_PWM_CHANNEL);
  ledcWrite(MOTOR_PWM_CHANNEL, duty);
#endif
}

#ifdef LIDAR_OUTPUT_BINARY
uint16_t crc16Ccitt(const uint8_t* data, size_t len, uint16_t crc = 0xFFFF) {
  for (size_t i = 0; i < len; i++) {
    crc ^= uint16_t(data[i]) << 8;
    for (int b = 0; b < 8; b++) crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
  }
  return crc;
}

// Frame layout, little-endian (see README.md):
//   sync 2 (A5 5A) | version 1 | seq 2 | scan_duration_us 4 | bins 2 |
//   distances_mm 2*bins | crc16 2 (CCITT-FALSE over everything after sync)
void sendBinaryFrame() {
  constexpr uint8_t FRAME_VERSION = 1;
  constexpr size_t HEADER = 11;
  static uint8_t frame[HEADER + 2 * SCAN_BINS + 2];

  frame[0] = 0xA5;
  frame[1] = 0x5A;
  frame[2] = FRAME_VERSION;
  frame[3] = scan_seq & 0xFF;
  frame[4] = scan_seq >> 8;
  for (int i = 0; i < 4; i++) frame[5 + i] = (scan_duration_us >> (8 * i)) & 0xFF;
  frame[9] = SCAN_BINS & 0xFF;
  frame[10] = SCAN_BINS >> 8;
  for (int i = 0; i < SCAN_BINS; i++) {
    frame[HEADER + 2 * i] = completed[i] & 0xFF;
    frame[HEADER + 2 * i + 1] = completed[i] >> 8;
  }
  const size_t crc_pos = HEADER + 2 * SCAN_BINS;
  const uint16_t crc = crc16Ccitt(frame + 2, crc_pos - 2);
  frame[crc_pos] = crc & 0xFF;
  frame[crc_pos + 1] = crc >> 8;

  usbWrite(frame, sizeof(frame));
}
#endif

void onScanComplete() {
  memcpy(completed, building, sizeof(completed));
  scan_seq++;
  scans_total++;
#ifdef LIDAR_OUTPUT_BINARY
  sendBinaryFrame();
#endif
}

void addPoint(const X2Parser::Point& p) {
  if (p.distance_mm == 0) return;
  int bin = int(p.angle_deg + 0.5f) % SCAN_BINS;
  // Several points can fall in the same bin: keep the closest one, which is the
  // conservative choice for obstacle avoidance.
  if (building[bin] == 0 || p.distance_mm < building[bin]) building[bin] = p.distance_mm;
}

void onPacket() {
  if (parser.isRingStart()) {
    const uint32_t now = micros();
    if (have_ring_start) {
      scan_duration_us = now - ring_start_us;
      onScanComplete();
    }
    ring_start_us = now;
    have_ring_start = true;
    lidar_reported_freq_hz = parser.ringScanFreqHz();
    memset(building, 0, sizeof(building));
  }
  for (uint8_t i = 0; i < parser.sampleCount(); i++) addPoint(parser.point(i));
}

#ifdef LIDAR_OUTPUT_DEBUG
void printDebug() {
  static unsigned long last_ms = 0;
  static uint32_t last_scans = 0;
  const unsigned long now = millis();
  if (now - last_ms < DEBUG_PERIOD_MS) return;

  const float scans_per_s = (scans_total - last_scans) * 1000.0f / (now - last_ms);
  last_ms = now;
  last_scans = scans_total;

  int valid = 0;
  for (int i = 0; i < SCAN_BINS; i++) valid += completed[i] != 0;

  char line[200];
  const int len = snprintf(
      line, sizeof(line),
      "scans/s %.2f (lidar %.1f Hz) | packets ok %lu, checksum err %lu, framing err %lu | "
      "valid bins %d/%d | 0:%u 90:%u 180:%u 270:%u mm | usb dropped %lu\n",
      scans_per_s, lidar_reported_freq_hz, (unsigned long)parser.packetsOk(),
      (unsigned long)parser.checksumErrors(), (unsigned long)parser.framingErrors(), valid,
      SCAN_BINS, completed[0], completed[90], completed[180], completed[270],
      (unsigned long)usb_dropped);
  usbWrite(reinterpret_cast<const uint8_t*>(line), min(len, int(sizeof(line)) - 1));
}
#endif

}  // namespace

void setup() {
  Serial.setTxBufferSize(USB_TX_BUFFER);
  Serial.begin(115200);  // USB CDC: the baud rate is ignored
  Serial.setTxTimeoutMs(0);  // never block on USB, see usbWrite()

  Serial1.setRxBufferSize(LIDAR_RX_BUFFER);
  Serial1.begin(LIDAR_BAUD, SERIAL_8N1, PIN_LIDAR_RX, -1);  // RX only

  setupMotor();
}

void loop() {
#ifdef LIDAR_OUTPUT_RAW
  uint8_t chunk[256];
  int n = min(Serial1.available(), int(sizeof(chunk)));
  n = min(n, Serial.availableForWrite());  // if the Pi is not reading, leave bytes in the UART buffer
  if (n > 0) Serial.write(chunk, Serial1.readBytes(chunk, n));
#else
  while (Serial1.available() > 0) {
    if (parser.feed(uint8_t(Serial1.read()))) onPacket();
  }
#endif

#ifdef LIDAR_OUTPUT_DEBUG
  printDebug();
#endif
}
