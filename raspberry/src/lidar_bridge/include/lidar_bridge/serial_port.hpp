#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace lidar_bridge
{

// Minimal POSIX serial port in raw mode. The ESP32-C3 uses native USB CDC, so
// the baud rate is ignored by the device but still has to be set.
class SerialPort
{
public:
  SerialPort() = default;
  ~SerialPort();
  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;

  // Throws std::runtime_error on failure.
  void open(const std::string & device);
  void close();
  bool isOpen() const {return fd_ >= 0;}

  // Waits up to timeout_ms for data. Returns the number of bytes read (0 on
  // timeout). Throws std::runtime_error if the device is gone.
  size_t read(uint8_t * buf, size_t len, int timeout_ms);

private:
  int fd_ = -1;
};

}  // namespace lidar_bridge
