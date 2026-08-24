#pragma once

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cstring>
#include <string>

namespace chassis {

// POSIX 串口封装:115200 8N1 原始模式,read 带超时(VTIME),线程安全度:
// read_thread 只读 fd,write 只在控制线程调用,互不竞争。
class SerialPort {
public:
  ~SerialPort()
  {
    close();
  }

  bool open(const std::string & device, const speed_t baud = B115200)
  {
    close();
    fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      return false;
    }

    termios tty{};
    if (tcgetattr(fd_, &tty) != 0) {
      close();
      return false;
    }
    cfmakeraw(&tty);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;  // 8 数据位
    tty.c_cflag &= ~(PARENB | CSTOPB);           // 无校验、1 停止位
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cc[VMIN] = 0;   // 非阻塞配合 VTIME
    tty.c_cc[VTIME] = 2;  // 200ms 读超时,读线程借此周期检查退出标志
    if (cfsetispeed(&tty, baud) != 0 || cfsetospeed(&tty, baud) != 0) {
      close();
      return false;
    }
    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
      close();
      return false;
    }
    tcflush(fd_, TCIOFLUSH);
    return true;
  }

  void close()
  {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  bool is_open() const { return fd_ >= 0; }

  // 返回实际读到的字节数(0 = 超时),负值 = 已关闭
  ssize_t read(uint8_t * buf, const size_t max_len)
  {
    if (fd_ < 0) {
      return -1;
    }
    return ::read(fd_, buf, max_len);
  }

  bool write(const uint8_t * buf, const size_t len)
  {
    if (fd_ < 0) {
      return false;
    }
    size_t sent = 0;
    while (sent < len) {
      const ssize_t n = ::write(fd_, buf + sent, len - sent);
      if (n < 0) {
        return false;
      }
      sent += static_cast<size_t>(n);
    }
    return true;
  }

private:
  int fd_ = -1;
};

}  // namespace chassis
