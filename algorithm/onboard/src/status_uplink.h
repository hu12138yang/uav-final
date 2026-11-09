#ifndef _ONBOARD_STATUS_UPLINK_H_
#define _ONBOARD_STATUS_UPLINK_H_

// 机载 -> 地面站的状态上报（独立 UDP 链路）。
//
// 协议里所有状态报文都是「飞机 -> 操控端」，没有机载计算机 -> 地面站这一向，
// 飞控也不会替我们转发。所以要回传推算位置，只能自己开一条链路。
// 这里按行发 JSON，地面站侧用 netcat / Python / 自写接收端都能看。

#include <cstdint>
#include <string>

namespace ego_onboard
{

  class StatusUplink
  {
  public:
    ~StatusUplink();

    // addr 形如 "192.168.1.100:14560" 或 "127.0.0.1:9999"
    bool open(const std::string &addr, std::string &err);
    void close();
    bool isOpen() const { return fd_ >= 0; }

    // 尽力而为：发失败不抛错、不阻塞
    bool sendLine(const std::string &line);

  private:
    int fd_ = -1;
    uint32_t addr_be_ = 0;
    uint16_t port_be_ = 0;
  };

} // namespace ego_onboard

#endif
