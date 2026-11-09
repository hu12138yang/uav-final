#include <data_handler/data_handler.h>
#include <sensor/frame_assembler.h>
#include <fstream>
#include <iterator>
#include <vector>
#include <cstdio>
#include <cstring>
namespace {
ego_sensor::FrameAssembler assembler;
int packets=0,frames=0;bool provenance=true;size_t points=0;
void callback(uint32_t,uint8_t,LivoxLidarEthernetPacket *p,void*) {
  ++packets;assembler.addPacket(*p,0);ego_sensor::LidarFrame frame;
  if(assembler.popFrame(frame)){++frames;points+=frame.points.size();provenance=provenance&&!frame.payload_crc_present;}
}
}
int main(int argc,char **argv) {
  if(argc!=2)return 1;
  std::ifstream f(argv[1],std::ios::binary);std::vector<uint8_t>b((std::istreambuf_iterator<char>(f)),std::istreambuf_iterator<char>());
  if(b.size()!=1380||b[10]!=1||b[5]!=96||b[24]||b[25]||b[26]||b[27])return 1;
  auto &sdk=livox::lidar::DataHandler::GetInstance();sdk.SetPointDataCallback(callback,nullptr);
  sdk.AllowZeroMid360sPointCrc(false);sdk.Handle(35,1,b.data(),b.size());const bool strict=packets==0;
  sdk.AllowZeroMid360sPointCrc(true);
  uint64_t initial=0;std::memcpy(&initial,b.data()+28,8);
  // Measured payload, controlled 0.5 ms timestamps to exercise full 100 ms assembly.
  for(uint64_t i=0;i<250;++i){const uint64_t t=initial+i*500000;std::memcpy(b.data()+28,&t,8);sdk.Handle(35,1,b.data(),b.size());}
  const bool received=packets==250&&frames>=1&&points>0&&provenance;
  sdk.AllowZeroMid360sPointCrc(false);sdk.Destory();
  std::printf("Measured Mid360S payload direct SDK->frame: strict=%d packets=%d frames=%d points=%zu CRC absence preserved=%d\n",strict,packets,frames,points,provenance);
  return strict&&received?0:1;
}