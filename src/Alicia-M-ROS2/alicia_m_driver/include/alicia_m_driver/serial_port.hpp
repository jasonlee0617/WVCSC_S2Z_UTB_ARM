#ifndef ALICIA_M_DRIVER__SERIAL_PORT_HPP_
#define ALICIA_M_DRIVER__SERIAL_PORT_HPP_

#include <string>
#include <vector>
#include <cstdint>
#include <termios.h>

namespace alicia_m_driver
{

/// 非阻塞串口通信类，使用 POSIX termios
class SerialPort
{
public:
  SerialPort();
  ~SerialPort();

  SerialPort(const SerialPort&) = delete;
  SerialPort& operator=(const SerialPort&) = delete;

  /// 打开串口设备
  /// @param device 设备路径，如 /dev/ttyACM0
  /// @param baudrate 波特率，默认 1000000
  /// @return 是否成功
  bool open(const std::string& device, int baudrate = 1000000);

  /// 关闭串口
  void close();

  /// 是否已打开
  bool is_open() const;

  /// 发送数据（完整写入）
  bool write(const std::vector<uint8_t>& data);
  bool write(const uint8_t* data, size_t len);

  /// 非阻塞读取，返回实际读取的字节数
  /// @param buf 输出缓冲区
  /// @param max_len 最大读取长度
  /// @param timeout_ms 超时毫秒数，0 表示立即返回
  ssize_t read(uint8_t* buf, size_t max_len, int timeout_ms = 0);

  /// 清空接收缓冲区
  void flush_input();

private:
  int fd_;
  static speed_t to_speed_t(int baudrate);
};

}  // namespace alicia_m_driver

#endif  // ALICIA_M_DRIVER__SERIAL_PORT_HPP_
