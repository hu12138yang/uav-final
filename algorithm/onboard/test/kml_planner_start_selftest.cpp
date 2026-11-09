#include <onboard/kml_planner_start.h>
#include "fake_fcu.h"
#include <cstdio>
using namespace ego_onboard;
struct CounterPlanner : PlannerIO {
 int steps=0,legs=0;std::vector<Eigen::Vector3d> reference;
 void setReferenceRoute(const std::vector<Eigen::Vector3d> &v)override{reference=v;}
 void setLeg(const Eigen::Vector3d &,const Eigen::Vector3d &,bool)override{++legs;}
 void onPose(const ego_sensor::TimedPose &)override{}
 void onScanFrame(const ego_sensor::LidarFrame &)override{}
 bool step(double,const Eigen::Vector3d &p,ego_planner::QuadrotorCommand &c)override{++steps;c.position=p;return true;}
};
int main(int argc,char **argv){if(argc!=2)return 1;int failures=0;auto check=[&](bool ok,const char *s){std::printf("[%s] %s\n",ok?"PASS":"FAIL",s);if(!ok)++failures;};
 MissionInputConfig cfg;cfg.file=argv[1];MissionInput input;input.start(cfg);while(input.state()!=MissionInputState::READY&&input.state()!=MissionInputState::FAILED)input.step();
 check(input.state()==MissionInputState::READY,"received KML parses before planner begins");if(input.state()!=MissionInputState::READY)return 1;
 const auto route=input.mission().route;CounterPlanner planner;GeoOrigin invalid;
 check(!bindKmlPlanner(route,invalid,planner)&&planner.legs==0,"KML task waits for a known HOME before geographic conversion");
 check(!preplanKml(false,true,true,Phase::LINK,1,Eigen::Vector3d::Zero(),planner)&&planner.steps==0,"no valid KML means no planner begin");
 check(!preplanKml(true,true,false,Phase::LINK,1,Eigen::Vector3d::Zero(),planner)&&planner.steps==0,"invalid inputs do not start trajectory computation");
 ego_test::FakeFcu fcu;fcu.lat0=route.front().lat_deg;fcu.lon0=route.front().lon_deg;fcu.pos.z()=50;fcu.main_mode=MODE_POS_HOLD;fcu.armed=false;fcu.rate_gpi=50;fcu.rate_att=100;
 OffboardPilot pilot;OffboardPilot::Config pc;pc.observe_only=true;pc.gps_min_fix=6;pilot.setConfig(pc);pilot.setPlanner(&planner);pilot.startObserving();
 bool bound=false,forbidden=false;ego_mav::Parser wire_parser;
 for(int i=0;i<100;++i){double now=i*.01;pilot.setReceiveTime(now);std::vector<std::vector<uint8_t>> down,up;fcu.tick(.01,now,down);for(const auto &b:down)pilot.onMavlinkBytes(b.data(),b.size());
  if(!bound)bound=bindKmlPlanner(route,pilot.origin(),planner);
  preplanKml(true,bound,pilot.poseFresh(now)&&pilot.gpsFixType()>=6,pilot.phase(),now,pilot.latestPose().p,planner);
  pilot.tick(now,up);for(const auto &b:up)for(auto byte:b){ego_mav::MavMessage m;if(wire_parser.push(byte,m)&&m.msgid!=0&&m.msgid!=66)forbidden=true;}
 }
 check(bound&&planner.legs==1&&planner.steps>0&&!pilot.armed()&&pilot.phase()==Phase::LINK,"valid KML begins planner before arming or mission ACK");
 check(planner.reference.size()==route.size()&&planner.reference.front().head<2>().norm()<1e-8&&planner.reference.front().z()==50,"KML ENU conversion uses actual HOME and requested relative height");
 check(!forbidden,"background planning never sends mission/mode/motion commands in observe mode");
 const int before=planner.steps;check(!preplanKml(true,true,true,Phase::PRE_OFFBOARD,2,Eigen::Vector3d::Zero(),planner)&&!preplanKml(true,true,true,Phase::OFFBOARD_CRUISE,2,Eigen::Vector3d::Zero(),planner)&&planner.steps==before,"handover and active flight retain existing planning/output ownership");
 return failures?1:0;}