#include "alicia_m_driver/serial_port.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
#include <cerrno>
#include <cstring>
#include <stdexcept>

namespace alicia_m_driver
{

SerialPort::SerialPort() : fd_(-1) {}

SerialPort::~SerialPort()
{
  close();
}

bool SerialPort::open(const std::string& device, int baudrate)
{
  if (fd_ >= 0) close();

  fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) return false;

  struct termios tty{};
  if (tcgetattr(fd_, &tty) != 0) {
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  speed_t spd = to_speed_t(baudrate);

  cfsetispeed(&tty, spd);
  cfsetospeed(&tty, spd);

  // 8N1, 无流控
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag |= CLOCAL | CREAD;

  // 原始模式输入
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

  tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
  tty.c_oflag &= ~OPOST;

  // 非阻塞读取
  tty.c_cc[VMIN]  = 0;
  tty.c_cc[VTIME] = 0;

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  // 清空缓冲区
  tcflush(fd_, TCIOFLUSH);

  return true;
}

void SerialPort::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool SerialPort::is_open() const
{
  return fd_ >= 0;
}

bool SerialPort::write(const std::vector<uint8_t>& data)
{
  return write(data.data(), data.size());
}

bool SerialPort::write(const uint8_t* data, size_t len)
{
  if (fd_ < 0 || len == 0) return false;

  size_t total_written = 0;
  while (total_written < len) {
    ssize_t n = ::write(fd_, data + total_written, len - total_written);
    if (n < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        // 短暂等待后重试
        usleep(100);
        continue;
      }
      return false;
    }
    total_written += static_cast<size_t>(n);
  }

  // 等待数据发送完成
  tcdrain(fd_);
  return true;
}

ssize_t SerialPort::read(uint8_t* buf, size_t max_len, int timeout_ms)
{
  if (fd_ < 0) return -1;

  if (timeout_ms > 0) {
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(fd_, &read_fds);

    struct timeval tv;
    tv.tv_sec  = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;

    int ret = select(fd_ + 1, &read_fds, nullptr, nullptr, &tv);
    if (ret <= 0) return 0;
  }

  ssize_t n = ::read(fd_, buf, max_len);
  if (n < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK) return 0;
    return -1;
  }
  return n;
}

void SerialPort::flush_input()
{
  if (fd_ >= 0) {
    tcflush(fd_, TCIFLUSH);
  }
}

speed_t SerialPort::to_speed_t(int baudrate)
{
  switch (baudrate) {
    case 9600:    return B9600;
    case 19200:   return B19200;
    case 38400:   return B38400;
    case 57600:   return B57600;
    case 115200:  return B115200;
    case 230400:  return B230400;
    case 460800:  return B460800;
    case 500000:  return B500000;
    case 576000:  return B576000;
    case 921600:  return B921600;
    case 1000000: return B1000000;
    case 1152000: return B1152000;
    case 1500000: return B1500000;
    case 2000000: return B2000000;
    default:      return B1000000;
  }
}

}  // namespace alicia_m_driver
