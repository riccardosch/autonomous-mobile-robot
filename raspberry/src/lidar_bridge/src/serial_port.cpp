#include "lidar_bridge/serial_port.hpp"

#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>

namespace lidar_bridge
{

namespace
{
std::runtime_error sysError(const std::string & what)
{
  return std::runtime_error(what + ": " + std::strerror(errno));
}
}  // namespace

SerialPort::~SerialPort() {close();}

void SerialPort::open(const std::string & device)
{
  close();
  fd_ = ::open(device.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    throw sysError("cannot open " + device);
  }

  termios tty{};
  if (tcgetattr(fd_, &tty) != 0) {
    const auto err = sysError("tcgetattr " + device);
    close();
    throw err;
  }
  cfmakeraw(&tty);
  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);
  tty.c_cflag |= CLOCAL | CREAD;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;
  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    const auto err = sysError("tcsetattr " + device);
    close();
    throw err;
  }
  // Linux raises DTR and RTS when the port is opened. On the ESP32-C3 USB
  // serial they drive the reset and boot pins: left raised, the board can stay
  // silent. Release both, as the tools that work with it do.
  setModemLines(TIOCM_DTR | TIOCM_RTS, false);
  tcflush(fd_, TCIFLUSH);  // drop whatever was queued before we started
}

void SerialPort::setModemLines(int lines, bool raised)
{
  ioctl(fd_, raised ? TIOCMBIS : TIOCMBIC, &lines);  // not fatal if unsupported
}

void SerialPort::resetDevice()
{
  // RTS raised with DTR released holds the ESP32 in reset; releasing RTS
  // lets it boot into the firmware.
  setModemLines(TIOCM_DTR, false);
  setModemLines(TIOCM_RTS, true);
  usleep(100 * 1000);
  setModemLines(TIOCM_RTS, false);
}

void SerialPort::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

size_t SerialPort::read(uint8_t * buf, size_t len, int timeout_ms)
{
  pollfd pfd{fd_, POLLIN, 0};
  const int ready = poll(&pfd, 1, timeout_ms);
  if (ready < 0) {
    if (errno == EINTR) {
      return 0;
    }
    throw sysError("poll");
  }
  if (ready == 0) {
    return 0;
  }
  if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
    throw std::runtime_error("serial device disconnected");
  }
  const ssize_t n = ::read(fd_, buf, len);
  if (n < 0) {
    if (errno == EAGAIN || errno == EINTR) {
      return 0;
    }
    throw sysError("read");
  }
  if (n == 0) {
    throw std::runtime_error("serial device disconnected");  // readable but EOF
  }
  return size_t(n);
}

}  // namespace lidar_bridge
