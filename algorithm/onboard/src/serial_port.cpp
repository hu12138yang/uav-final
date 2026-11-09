#include "serial_port.h"

#include <cerrno>
#include <cstring>

#include <fcntl.h>
#include <poll.h>
#include <chrono>
#include <sys/socket.h>
#include <netdb.h>
#include <netinet/tcp.h>
#include <termios.h>
#include <unistd.h>

namespace ego_onboard
{

  static speed_t baudToSpeed(int baud)
  {
    switch (baud)
    {
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    default: return 0;
    }
  }

  SerialPort::~SerialPort() { close(); }

  bool SerialPort::open(const std::string &dev, int baud, std::string &err)
  {
    close();
    const bool tcp = dev.compare(0, 6, "tcp://") == 0;
    if (tcp) {
      const auto colon = dev.find_last_of(':');
      if (colon <= 6 || colon == std::string::npos) { err="expected tcp://host:port"; return false; }
      const std::string host=dev.substr(6, colon-6), port=dev.substr(colon+1);
      addrinfo hints{}, *res=nullptr; hints.ai_socktype=SOCK_STREAM;
      if (getaddrinfo(host.c_str(), port.c_str(), &hints, &res)) { err="TCP bridge address lookup failed"; return false; }
      for (auto *a=res; a; a=a->ai_next) {
        fd_=::socket(a->ai_family, a->ai_socktype | SOCK_NONBLOCK, a->ai_protocol);
        if (fd_<0) continue;
        int rc=::connect(fd_,a->ai_addr,a->ai_addrlen);
        pollfd pfd{fd_, POLLOUT, 0}; int e=0; socklen_t n=sizeof(e);
        if (rc==0 || (errno==EINPROGRESS && ::poll(&pfd,1,1000)>0 && getsockopt(fd_,SOL_SOCKET,SO_ERROR,&e,&n)==0 && e==0)) break;
        ::close(fd_); fd_=-1;
      }
      freeaddrinfo(res);
      if(fd_>=0) { int yes=1; setsockopt(fd_,IPPROTO_TCP,TCP_NODELAY,&yes,sizeof(yes)); }
    } else fd_ = ::open(dev.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0)
    {
      err = std::string("open(") + dev + ") 失败: " + std::strerror(errno);
      return false;
    }

    if (!tcp) {
    termios tio;
    std::memset(&tio, 0, sizeof(tio));
    if (tcgetattr(fd_, &tio) != 0)
    {
      err = std::string("tcgetattr 失败: ") + std::strerror(errno);
      close();
      return false;
    }
    cfmakeraw(&tio); // 8N1、无回显、无行缓冲
    const speed_t spd = baudToSpeed(baud);
    if (!spd) { err="unsupported baud rate"; close(); return false; }
    cfsetispeed(&tio, spd);
    cfsetospeed(&tio, spd);
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CRTSCTS; // 不用硬件流控
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;
    if (tcsetattr(fd_, TCSANOW, &tio) != 0)
    {
      err = std::string("tcsetattr 失败: ") + std::strerror(errno);
      close();
      return false;
    }
    tcflush(fd_, TCIOFLUSH);
    }

    run_ = true;
    reader_ = std::thread([this]() {
      uint8_t buf[1024];
      while (run_)
      {
        // Wait for readiness rather than adding a fixed 2 ms to each response.
        pollfd ready{fd_, POLLIN, 0};
        const int rc = ::poll(&ready, 1, 10);
        if (rc < 0 && errno == EINTR) continue;
        if (rc < 0) break;
        if (rc == 0) continue;
        if (!(ready.revents & POLLIN) && (ready.revents & (POLLERR | POLLHUP | POLLNVAL))) break;
        const ssize_t n = ::read(fd_, buf, sizeof(buf));
        if (n > 0)
        {
          std::lock_guard<std::mutex> lk(mtx_);
          rx_.insert(rx_.end(), buf, buf + n);
          if (rx_.size() > (1u << 20)) // 没人取走时别无限涨
            rx_.erase(rx_.begin(), rx_.begin() + (rx_.size() - (1u << 20)));
        }
        else if (n == 0 || (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR))
          break;
      }
      run_ = false;
    });
    return true;
  }

  void SerialPort::close()
  {
    run_ = false;
    if (reader_.joinable())
      reader_.join();
    if (fd_ >= 0)
    {
      ::close(fd_);
      fd_ = -1;
    }
    std::lock_guard<std::mutex> lk(mtx_);
    rx_.clear();
  }

  void SerialPort::take(std::vector<uint8_t> &out)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    out.swap(rx_);
    rx_.clear();
  }

  bool SerialPort::write(const uint8_t *data, size_t len)
  {
    if (fd_ < 0 || !run_)
      return false;
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::milliseconds(20);
    size_t off = 0;
    while (off < len)
    {
      const ssize_t n = ::write(fd_, data + off, len - off);
      if (n < 0 && errno == EINTR) continue;
      if (n > 0)
      {
        off += (size_t)n;
        continue;
      }
      if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
      {
        if (std::chrono::steady_clock::now() >= deadline) return false;
        pollfd pfd{fd_, POLLOUT, 0};
        if (::poll(&pfd,1,2)<0 && errno!=EINTR) return false;
        continue;
      }
      return false;
    }
    return true;
  }

} // namespace ego_onboard
