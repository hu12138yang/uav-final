#include <onboard/time_alignment.h>
#include <sensor/frame_assembler.h>
#include <sensor/scan_assembler.h>
#include <onboard/offboard_pilot.h>
#include <cstdio>
#include <cstring>
using namespace ego_onboard;
using namespace ego_sensor;
using namespace ego_mav;
namespace {
int failures=0;
void check(bool ok,const char *name) {std::printf("[%s] %s\n",ok?"PASS":"FAIL",name);if(!ok)++failures;}
std::vector<uint8_t> packet(uint64_t ns,uint8_t type=1) {
  // Independent protocol fixture: THREE points, first-to-last span 10us.
  std::vector<uint8_t> b(36+3*14,0);
  b[1]=b.size(); b[3]=100; b[5]=3; b[10]=1; b[11]=type;
  for(int i=0;i<8;++i)b[28+i]=(ns>>(8*i))&255;
  for(int i=0;i<3;++i) {b[36+i*14]=0xE8;b[37+i*14]=0x03;}
  return b;
}
}
int main() {
  TimeAlignment t;
  for(int i=0;i<10;++i) {
    const double sent=100+i*.2;
    const auto nonce=t.request(sent);
    check(t.accept(int64_t((sent+.002-42)*1e9),nonce,sent+.004),"accept bounded RTT response");
  }
  check(t.valid(102)&&std::abs(t.hostTime(60)-102)<1e-6,"FCU boot -> monotonic offset is midpoint, not receive time");
  check(t.bootMs(102)==60000,"outgoing setpoint uses FCU boot clock");
  check(!t.valid(106),"stale clock mapping expires");
  auto nonce=t.request(110);
  check(!t.accept(int64_t(68.02e9),nonce,110.04),"reject high RTT");
  nonce=t.request(111);
  check(!t.accept(int64_t(69e9),nonce+1,111.002),"reject another request's nonce");
  nonce=t.request(112);
  const auto generation=t.generation(); t.accept(int64_t(1e9),nonce,112.002);
  check(!t.valid(112.002)&&t.generation()>generation,"FCU reboot / offset jump invalidates mapping");
  EpochBridge epoch;check(epoch.sample(20,1700000020),"initialize epoch bridge");
  check(std::abs(epoch.toMonotonic(1700000021)-21)<1e-9,"PTP Unix -> monotonic");
  check(!epoch.sample(21,1700000022),"detect host realtime step");
  auto b=packet(1000000000); auto &p=*reinterpret_cast<LivoxLidarEthernetPacket*>(b.data());
  LivoxParser parser; LidarFrame f;
  check(parser.parsePacket(p,0,f)&&f.points.size()==3,"parse literal real-protocol payload");
  check(std::abs(f.points[1].t_rel-5e-6)<1e-12&&std::abs(f.points[2].t_rel-10e-6)<1e-12,"3-point packet spans exactly 10us, middle point 5us");
  p.length=36;LidarFrame bad;
  check(!parser.parsePacket(p,0,bad)&&bad.points.empty(),"reject truncated point payload");
  p.length=b.size();
  FrameAssembler assembler; FrameAssembler::Config cfg; cfg.frame_duration=.00001;assembler.setConfig(cfg);
  assembler.addPacket(p,0);check(assembler.popFrame(f),"frame completes using last-point timestamp");
  b=packet(100000000,0);assembler.addPacket(*reinterpret_cast<LivoxLidarEthernetPacket*>(b.data()),0);
  check(assembler.droppedPackets()==1&&assembler.popFrame(f)&&f.time_type==0&&f.t_base<.2,"restart/source switch flushes old scan and immediately recovers");
  // Spherical convention from Mid-360S manual p20: theta polar, phi azimuth.
  std::vector<uint8_t> sphere(36+sizeof(LivoxLidarSpherPoint),0);
  auto &sp=*reinterpret_cast<LivoxLidarEthernetPacket*>(sphere.data());sp.length=sphere.size();sp.dot_num=1;sp.data_type=3;
  auto &point=*reinterpret_cast<LivoxLidarSpherPoint*>(sp.data);point.depth=1000;point.theta=9000;
  LidarFrame sf;check(parser.parsePacket(sp,0,sf)&&sf.points.size()==1&&(sf.points[0].p-Eigen::Vector3d(1,0,0)).norm()<1e-9,"spherical polar=90deg azimuth=0 maps to +X");
  PoseBuffer poses;poses.setStrictCoverage(true);TimedPose a;a.t=1;poses.add(a);a.t=1.01;poses.add(a);
  TimedPose out;check(!poses.poseAt(1.011,out),"strict live pose buffer never clamps missing future pose");
  a.t=2;poses.add(a);check(!poses.poseAt(1.5,out),"interpolation cannot bridge a telemetry outage");
  ScanAssembler scan;std::vector<Eigen::Vector3d> cloud;LidarFrame empty;
  check(!scan.toWorld(empty,poses,cloud),"empty cloud cannot count as a healthy map update");
  GeoOrigin origin; origin.valid=true;
  PoseAssembler fused; fused.setOrigin(origin); PoseAssemblerConfig strict; strict.strict_position_brackets=true; fused.setConfig(strict);
  GlobalPositionInt g;g.time_boot_ms=1000;g.relative_alt=1000;fused.onGlobalPositionInt(g);
  Attitude att;att.time_boot_ms=1010;TimedPose fused_pose;
  check(!fused.onAttitude(att,1.01,fused_pose),"physical pose waits for a future RTK/EKF bracket");
  g.time_boot_ms=1020;g.relative_alt=3000;fused.onGlobalPositionInt(g);
  check(fused.onAttitude(att,1.01,fused_pose)&&std::abs(fused_pose.p.z()-2.0)<1e-9,"RTK/EKF position interpolates to the attitude sample time");
  // Observation must remain non-controlling even with fresh, armed FCU telemetry.
  OffboardPilot pilot;OffboardPilot::Config pc;pc.require_timesync=true;pilot.setConfig(pc);pilot.startObserving();
  bool forbidden=false;Parser decoder;
  for(double now=100;now<102;now+=.01) {
    std::vector<std::vector<uint8_t>> frames;pilot.tick(now,frames);
    for(const auto &wire:frames) for(auto byte:wire) {MavMessage msg;if(decoder.push(byte,msg)) {
      if(msg.msgid==76) { uint8_t p[33]; padPayload(msg.payload.data(),msg.payload.size(),p,33); if(rdU16(p,28)!=512||rdF32(p,0)!=242) forbidden=true; }
      else if(msg.msgid!=0&&msg.msgid!=111&&msg.msgid!=66) forbidden=true;
    }}
  }
  check(!forbidden,"observe path only emits heartbeat / clock / telemetry requests");
  std::printf("timing_selftest: %d failures\n",failures);return failures?1:0;
}
