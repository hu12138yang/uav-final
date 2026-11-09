#include "status_uplink.h"

#include <cerrno>
#include <cstdlib>
#include <cstring>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

namespace ego_onboard
{

  StatusUplink::~StatusUplink() { close(); }

  bool StatusUplink::open(const std::string &addr, std::string &err)
  {
    close();

    const size_t colon = addr.rfind(':');
    if (colon == std::string::npos)
    {
      err = "上报地址应为 ip:port，例如 192.168.1.100:14560";
      return false;
    }
    const std::string ip = addr.substr(0, colon);
    const int port = std::atoi(addr.c_str() + colon + 1);
    if (port <= 0 || port > 65535)
    {
      err = "端口非法: " + addr.substr(colon + 1);
      return false;
    }

    sockaddr_in sa;
    std::memset(&sa, 0, sizeof(sa));
    sa.sin_family = AF_INET;
    sa.sin_port = htons((uint16_t)port);
    if (inet_pton(AF_INET, ip.c_str(), &sa.sin_addr) != 1)
    {
      err = "IP 地址非法: " + ip;
      return false;
    }

    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0)
    {
      err = std::string("socket() 失败: ") + std::strerror(errno);
      return false;
    }
    addr_be_ = sa.sin_addr.s_addr;
    port_be_ = sa.sin_port;
    return true;
  }

  void StatusUplink::close()
  {
    if (fd_ >= 0)
    {
      ::close(fd_);
      fd_ = -1;
    }
  }

  bool StatusUplink::sendLine(const std::string &line)
  {
    if (fd_ < 0)
      return false;
    sockaddr_in sa;
    std::memset(&sa, 0, sizeof(sa));
    sa.sin_family = AF_INET;
    sa.sin_addr.s_addr = addr_be_;
    sa.sin_port = port_be_;
    const ssize_t n = ::sendto(fd_, line.data(), line.size(), MSG_DONTWAIT,
                               reinterpret_cast<sockaddr *>(&sa), sizeof(sa));
    return n == (ssize_t)line.size();
  }

} // namespace ego_onboard
