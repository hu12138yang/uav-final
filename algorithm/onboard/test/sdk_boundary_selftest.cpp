#include <data_handler/data_handler.h>
#include <livox_lidar_def.h>
#include <cstdint>
#include <cstdio>
#include <vector>
#include <cstring>
namespace {int calls=0;void callback(uint32_t,uint8_t,LivoxLidarEthernetPacket*,void*){++calls;}}
int main(){
  auto &handler=livox::lidar::DataHandler::GetInstance();
  handler.SetPointDataCallback(callback,nullptr);
  // Timestamp+single 14-byte Cartesian point. Python zlib oracle CRC=0x...
  std::vector<uint8_t> b(50,0);b[1]=50;b[5]=1;b[10]=1;
  const uint32_t oracle=0xA829B1E0; // replaced below from independent zlib fixture
  for(int i=0;i<4;++i)b[24+i]=(oracle>>(8*i))&255;
  handler.Handle(35,1,b.data(),b.size());
  const bool valid=calls==1;
  b.back()^=1;handler.Handle(35,1,b.data(),b.size());
  handler.Handle(35,1,b.data(),35);
  handler.Handle(35,1,nullptr,100);
  b[1]=51;handler.Handle(35,1,b.data(),b.size());
  const bool rejected=calls==1;
  std::printf("SDK boundary valid CRC=%d, corrupt/truncated/null rejected=%d\n",valid,rejected);
  // Reproduce the measured 1380-byte Mid360S zero-CRC packet shape.
  std::vector<uint8_t> raw(1380,0); raw[1]=0x64;raw[2]=5;raw[5]=96;raw[10]=1;
  handler.Handle(35,1,raw.data(),raw.size());
  const bool strict_zero_rejected=calls==1;
  handler.AllowZeroMid360sPointCrc(true);
  handler.Handle(35,1,raw.data(),raw.size());
  const bool observation_received=calls==2;
  raw[24]=1;handler.Handle(35,1,raw.data(),raw.size()); // nonzero bad CRC still rejected
  raw[24]=0;handler.Handle(35,1,raw.data(),35);
  handler.Handle(34,1,raw.data(),raw.size()); // other models not allowed
  raw[10]=0;handler.Handle(35,1,raw.data(),raw.size()); // zero-CRC IMU not allowed
  raw[10]=1;raw[5]=95;handler.Handle(35,1,raw.data(),raw.size());
  const bool diagnostic_boundaries=calls==2;
  raw[5]=96;handler.AllowZeroMid360sPointCrc(false);
  handler.Handle(35,1,raw.data(),raw.size());
  const bool strict_restored=calls==2;
  std::printf("Zero CRC strict rejection=%d observation=%d boundaries=%d strict restored=%d\n",
    strict_zero_rejected,observation_received,diagnostic_boundaries,strict_restored);
  handler.Destory();
  return valid&&rejected&&strict_zero_rejected&&observation_received&&diagnostic_boundaries&&strict_restored?0:1;
}
