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
struct PoseHistory : PlannerIO {
  std::vector<TimedPose> values;
  void onPose(const TimedPose &pose) override { values.push_back(pose); }
  void onScanFrame(const LidarFrame &) override {}
  bool step(double,const Eigen::Vector3d &,ego_planner::QuadrotorCommand &) override { return false; }
};
void feed(OffboardPilot &pilot,double now,uint32_t id,const uint8_t *payload,size_t size) {
  const auto wire=pack(1,1,0,id,payload,size);
  pilot.setReceiveTime(now);pilot.onMavlinkBytes(wire.data(),wire.size());
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
  PoseAssembler fused; fused.setOrigin(origin);
  GlobalPositionInt g;g.time_boot_ms=1000;g.relative_alt=1000;
  g.vx=100;g.vy=200;g.vz=-300;fused.onGlobalPositionInt(g);
  Attitude att;att.time_boot_ms=1010;TimedPose fused_pose;
  check(fused.onAttitude(att,1.01,fused_pose)&&
        (fused_pose.p-Eigen::Vector3d(.02,.01,1.03)).norm()<1e-9,
        "one position+NED velocity immediately predicts ENU position without a second fix");
  check(fused.onAttitude(att,1.20,fused_pose)&&std::abs(fused_pose.p.z()-1.60)<1e-9&&
        fused.onAttitude(att,1.25,fused_pose)&&std::abs(fused_pose.p.z()-1.75)<1e-9,
        "same real fix continuously predicts successive targets through its horizon");
  check(!fused.onAttitude(att,1.251,fused_pose)&&!fused.onAttitude(att,.99,fused_pose),
        "expired anchor and target before first fix are rejected instead of clamping time");
  g.time_boot_ms=1020;g.relative_alt=3000;g.vz=-100;fused.onGlobalPositionInt(g);
  check(fused.onAttitude(att,1.03,fused_pose)&&std::abs(fused_pose.p.z()-3.01)<1e-9,
        "new fix replaces both position and velocity anchor; no blend with old prediction");
  check(fused.onAttitude(att,1.01,fused_pose)&&std::abs(fused_pose.p.z()-1.03)<1e-9,
        "delayed attitude uses its latest causal anchor, never a future fix");
  g.time_boot_ms=1010;g.relative_alt=99000;fused.onGlobalPositionInt(g);
  check(fused.onAttitude(att,1.03,fused_pose)&&std::abs(fused_pose.p.z()-3.01)<1e-9,
        "out-of-order position cannot overwrite the latest real anchor");

  // At a real fix correction, point positions must not linearly mix the old/new anchors.
  PoseBuffer anchored;anchored.setStrictCoverage(true);
  TimedPose before,after;before.t=10.1;after.t=10.2;
  before.position_anchor.valid=true;before.position_anchor.t=10;
  before.position_anchor.v=Eigen::Vector3d(2,0,0);before.position_anchor.max_extrapolation_s=.3;
  before.position_anchor.positionAt(before.t,before.p);
  after.position_anchor=before.position_anchor;after.position_anchor.t=10.15;
  after.position_anchor.p=Eigen::Vector3d(10,0,0);after.position_anchor.v=Eigen::Vector3d(3,0,0);
  after.position_anchor.positionAt(after.t,after.p);
  anchored.add(before);anchored.add(after);
  check(anchored.poseAt(10.14,out)&&std::abs(out.p.x()-.28)<1e-9&&
        anchored.poseAt(10.16,out)&&std::abs(out.p.x()-10.03)<1e-9,
        "per-point lookup extrapolates its causal fix and switches at the new fix timestamp");
  LidarFrame prediction_frame;prediction_frame.t_base=10.14;
  LidarPoint sample;sample.p=Eigen::Vector3d(1,0,0);prediction_frame.points.push_back(sample);
  sample.t_rel=.02;prediction_frame.points.push_back(sample);
  check(scan.toWorld(prediction_frame,anchored,cloud)&&cloud.size()==2&&
        std::abs(cloud[0].x()-1.28)<1e-9&&std::abs(cloud[1].x()-11.03)<1e-9,
        "actual point transform uses velocity predictions on both sides of a real position correction");
  PoseBuffer stale;stale.setStrictCoverage(true);
  before.t=10.2;after.t=10.3;after.position_anchor=before.position_anchor;
  before.position_anchor.max_extrapolation_s=.25;after.position_anchor.max_extrapolation_s=.25;
  stale.add(before);stale.add(after);
  check(!stale.poseAt(10.26,out),"fresh attitude endpoints cannot renew an expired position anchor");

  // Exercise the real TIMESYNC-enabled pilot, not only the standalone position helper.
  OffboardPilot live;OffboardPilot::Config live_config;live_config.require_timesync=true;
  live.setConfig(live_config);live.startObserving();PoseHistory actual;live.setPlanner(&actual);
  uint8_t home[60]{};feed(live,100,242,home,sizeof(home));
  for(int i=0;i<10;++i) {
    double sent=100+i*.21;std::vector<std::vector<uint8_t>> requests;live.tick(sent,requests);
    Parser request_parser;
    for(const auto &wire:requests)for(auto byte:wire){MavMessage message;if(request_parser.push(byte,message)&&message.msgid==111){
      Timesync reply;Timesync::decode(message.payload.data(),message.payload.size(),reply);
      reply.tc1=int64_t((sent+.002-42)*1e9);
      reply.target_system=live_config.sysid;reply.target_component=live_config.compid;
      uint8_t payload[Timesync::LEN]{};
      reply.encode(payload);feed(live,sent+.004,111,payload,sizeof(payload));
    }}
  }
  check(live.clockReady(102),"real pilot fixture has a valid flight-clock mapping");
  uint8_t pos[28]{};wrU32(pos,0,60000);wrI32(pos,16,10000);wrU16(pos,24,uint16_t(int16_t(-200)));
  feed(live,102.002,33,pos,sizeof(pos));
  uint8_t attitude[28]{};
  for(uint32_t boot:{60010u,60050u,60100u}) {
    wrU32(attitude,0,boot);feed(live,42+boot*.001+.002,30,attitude,sizeof(attitude));
  }
  check(actual.values.size()==3&&std::abs(actual.values.back().p.z()-10.2)<1e-8&&
        std::abs(actual.values.back().position_anchor.t-102)<1e-7,
        "TIMESYNC-enabled pilot publishes every attitude from one fix and maps anchor time too");
  wrU32(pos,0,60120);wrI32(pos,16,20000);wrU16(pos,24,uint16_t(int16_t(-100)));
  feed(live,102.122,33,pos,sizeof(pos));wrU32(attitude,0,60130);
  feed(live,102.132,30,attitude,sizeof(attitude));
  check(actual.values.size()==4&&std::abs(actual.values.back().p.z()-20.01)<1e-8&&
        std::abs(actual.values.front().p.z()-10.02)<1e-8,
        "new fix immediately anchors the next attitude and preserves already-published history");
  wrU32(attitude,0,60371);feed(live,102.373,30,attitude,sizeof(attitude));
  check(actual.values.size()==4&&!live.poseFresh(102.373),
        "position dropout expires despite continued attitude traffic");
  wrU32(pos,0,60380);wrI32(pos,16,30000);feed(live,102.382,33,pos,sizeof(pos));
  wrU32(attitude,0,60390);feed(live,102.392,30,attitude,sizeof(attitude));
  check(actual.values.size()==5&&std::abs(actual.values.back().p.z()-30.01)<1e-8,
        "next real position recovers prediction immediately without a second-fix wait");
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
