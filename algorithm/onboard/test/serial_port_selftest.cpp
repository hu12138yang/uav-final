#include "serial_port.h"
#include <pty.h>
#include <unistd.h>
#include <fcntl.h>
#include <chrono>
#include <thread>
#include <cstdio>
using namespace ego_onboard;
int main() {
  int master=-1,slave=-1;char name[128];
  if(openpty(&master,&slave,name,nullptr,nullptr)) return 1;
  close(slave);fcntl(master,F_SETFL,O_NONBLOCK);
  SerialPort port;std::string error;
  if(!port.open(name,921600,error)) {std::puts(error.c_str());close(master);return 1;}
  std::vector<uint8_t> sent(8192),received,chunk;
  for(size_t i=0;i<sent.size();++i)sent[i]=uint8_t(i*37);
  size_t off=0;const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(1);
  while((off<sent.size()||received.size()<sent.size())&&std::chrono::steady_clock::now()<deadline) {
    if(off<sent.size()){ssize_t n=::write(master,sent.data()+off,sent.size()-off);if(n>0)off+=size_t(n);}
    port.take(chunk);received.insert(received.end(),chunk.begin(),chunk.end());
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  bool rx=sent==received;
  const bool wrote=port.write(sent.data(),128);std::vector<uint8_t> reply;
  for(int i=0;i<200&&reply.size()<128;++i) {
    uint8_t data[128];const ssize_t n=read(master,data,sizeof(data));if(n>0)reply.insert(reply.end(),data,data+n);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  bool tx=wrote&&reply.size()==128&&std::equal(reply.begin(),reply.end(),sent.begin());
  close(master);std::this_thread::sleep_for(std::chrono::milliseconds(50));
  bool disconnect=!port.isOpen()&&!port.write(sent.data(),1);port.close();
  std::printf("PTY receive bytes=%zu exact=%d transmit=%d disconnected write rejected=%d\n",received.size(),rx,tx,disconnect);
  return rx&&tx&&disconnect?0:1;
}
