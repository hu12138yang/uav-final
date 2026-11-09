#include <onboard/ego_planner_adapter.h>
#include "fake_fcu.h"
#include <cstdio>
using namespace ego_onboard;
int main() {
  int fail=0;
  auto check=[&](bool ok,const char *s){std::printf("[%s] %s\n",ok?"PASS":"FAIL",s);if(!ok)++fail;};
  EgoPlannerAdapter adapter;EgoPlannerAdapterConfig cfg;cfg.strict_pose_coverage=true;
  cfg.sensing_timeout_s=.5;adapter.init(cfg);adapter.prepareMap(50);
  check(!adapter.sensingHealthy(),"no first scan means sensing unhealthy even when map exists");
  ego_sensor::TimedPose pose;pose.t=1;pose.p.z()=50;adapter.onPose(pose);pose.t=1.01;adapter.onPose(pose);
  ego_sensor::LidarFrame frame;frame.t_base=1;ego_sensor::LidarPoint point;point.p.x()=5;point.t_rel=.005;frame.points.push_back(point);
  adapter.onScanFrame(frame);check(adapter.sensingHealthy(),"covered nonempty scan makes map ready before handover");
  frame.points[0].p.x()=.5;adapter.onScanFrame(frame);
  adapter.setLeg(Eigen::Vector3d(0,0,50),Eigen::Vector3d(8,0,50),false);
  ego_planner::QuadrotorCommand blocked_command;
  const bool held=adapter.step(1.01,pose.p,blocked_command);
  check(held&&adapter.stats().blocked_start_holds>0&&adapter.stats().replan==0&&
    blocked_command.position==pose.p&&blocked_command.velocity.isZero()&&
    blocked_command.trajectory_flag==ego_planner::TRAJECTORY_STATUS_EMPTY,
    "occupied start yields immediate finite HOLD without trajectory optimization");
  pose.t=2;adapter.onPose(pose);check(!adapter.sensingHealthy(),"scan silence expires map health even without another callback");
  frame.points.clear();frame.t_base=2;adapter.onScanFrame(frame);check(!adapter.sensingHealthy(),"empty scan cannot refresh stale map");
  // Protocol state machine: receiving a position once is not permanent validity.
  ego_test::FakeFcu fcu;fcu.takeoff_alt=50;
  OffboardPilot pilot;OffboardPilot::Config pc;pc.sensing_failsafe=false;pilot.setConfig(pc);
  pilot.start({{21,113,50},{21.0001,113,50}});
  double now=0;
  for(;now<40&&pilot.phase()!=Phase::OFFBOARD_CRUISE;now+=.01) {
    if(pilot.phase()==Phase::WAIT_GROUND_START)fcu.simulateGroundStationTakeoff();
    pilot.setReceiveTime(now);std::vector<std::vector<uint8_t>> down,up,response;fcu.tick(.01,now,down);
    for(const auto &wire:down)pilot.onMavlinkBytes(wire.data(),wire.size());
    pilot.tick(now,up);for(const auto &wire:up)fcu.onBytes(wire.data(),wire.size(),response);
    for(const auto &wire:response)pilot.onMavlinkBytes(wire.data(),wire.size());
  }
  check(pilot.phase()==Phase::OFFBOARD_CRUISE,"guard fixture reaches Offboard through GCS takeoff");
  std::vector<std::vector<uint8_t>> up;pilot.tick(now+.5,up);
  bool sp=false;ego_mav::Parser parser;
  for(const auto &wire:up)for(auto byte:wire){ego_mav::MavMessage m;if(parser.push(byte,m)&&m.msgid==84)sp=true;}
  check(pilot.phase()==Phase::FAILED&&!sp,"stale pose stops motion output and delegates to FCU offboard-loss handling");
  return fail?1:0;
}
