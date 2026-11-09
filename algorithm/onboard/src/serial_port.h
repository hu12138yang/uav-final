#ifndef _ONBOARD_SERIAL_PORT_H_
#define _ONBOARD_SERIAL_PORT_H_

// POSIX 串口收发：读线程把收到的字节攒进缓冲，主循环定期取走。
// 用途：机载计算机用 USB 串口直连飞控（硬件在环 / 实飞）。

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace ego_onboard
{

  class SerialPort
  {
  public:
    ~SerialPort();

    // baud 常用 57600 / 115200 / 230400 / 460800 / 921600
    bool open(const std::string &dev, int baud, std::string &err);
    void close();
    bool isOpen() const { return fd_ >= 0 && run_; }

    // 取走并清空已收到的字节（无数据则 out 为空）
    void take(std::vector<uint8_t> &out);

    bool write(const uint8_t *data, size_t len);

  private:
    int fd_ = -1;
    std::atomic<bool> run_{false};
    std::thread reader_;
    std::mutex mtx_;
    std::vector<uint8_t> rx_;
  };

} // namespace ego_onboard

#endif
