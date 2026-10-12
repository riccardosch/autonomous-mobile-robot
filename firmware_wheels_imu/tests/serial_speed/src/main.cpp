// Serial link speed test for the micro-ROS transport (test T7).
//
// At BASE_BAUD it waits for a line "baud <rate>", answers "ok <rate>" and
// switches to that rate. It then sends back every byte it receives until
// nothing arrives for IDLE_RETURN_MS, goes back to BASE_BAUD and prints how
// many bytes it echoed. Driven by serial_speed.py on the Pi.

#include <Arduino.h>

namespace {

constexpr unsigned long BASE_BAUD = 115200;
constexpr size_t BUFFER_BYTES = 4096;  // UART driver buffers, each direction
constexpr unsigned long IDLE_RETURN_MS = 1000;

char line[32];
size_t line_len = 0;

unsigned long echo_until_idle() {
  uint8_t buf[256];
  unsigned long total = 0;
  unsigned long last_ms = millis();
  while (millis() - last_ms < IDLE_RETURN_MS) {
    const size_t n = min(static_cast<size_t>(Serial.available()), sizeof(buf));
    if (n == 0) continue;
    Serial.readBytes(buf, n);
    Serial.write(buf, n);
    total += n;
    last_ms = millis();
  }
  return total;
}

void run_command(const char *cmd) {
  unsigned long baud = 0;
  if (sscanf(cmd, "baud %lu", &baud) != 1 || baud == 0) return;
  Serial.printf("ok %lu\n", baud);
  Serial.flush();
  Serial.updateBaudRate(baud);
  const unsigned long echoed = echo_until_idle();
  Serial.flush();
  Serial.updateBaudRate(BASE_BAUD);
  Serial.printf("echoed %lu bytes at %lu baud\n", echoed, baud);
}

}  // namespace

void setup() {
  Serial.setRxBufferSize(BUFFER_BYTES);
  Serial.setTxBufferSize(BUFFER_BYTES);
  Serial.begin(BASE_BAUD);
  Serial.println("serial speed test ready");
}

void loop() {
  while (Serial.available() > 0) {
    const char c = Serial.read();
    if (c == '\n' || c == '\r') {
      line[line_len] = '\0';
      if (line_len > 0) run_command(line);
      line_len = 0;
    } else if (line_len < sizeof(line) - 1) {
      line[line_len++] = c;
    }
  }
}
