#ifndef ONBOARD_KML_PLANNER_START_H
#define ONBOARD_KML_PLANNER_START_H
#include <onboard/mission_input.h>
namespace ego_onboard {
// Valid KML begins background planning. These helpers never send FCU messages.
inline bool bindKmlPlanner(const std::vector<GeoWaypoint> &route,const ego_mav::GeoOrigin &home,PlannerIO &planner) {
  if(route.size()<2||!home.valid)return false;
  std::vector<Eigen::Vector3d> enu;enu.reserve(route.size());
  for(const auto &w:route){auto xy=ego_mav::geoToEnuXY(w.lat_deg,w.lon_deg,home);enu.emplace_back(xy.x(),xy.y(),w.alt_rel_m);}
  planner.setReferenceRoute(enu);planner.setLeg(enu[0],enu[1],enu.size()==2);return true;
}
inline bool preplanKml(bool kml_ready,bool route_bound,bool inputs_ready,Phase phase,double now,
                      const Eigen::Vector3d &position,PlannerIO &planner) {
  if(!kml_ready||!route_bound||!inputs_ready)return false;
  if(phase!=Phase::LINK&&phase!=Phase::MISSION_UPLOAD&&phase!=Phase::WAIT_GROUND_START)return false;
  ego_planner::QuadrotorCommand preview;planner.step(now,position,preview);return true;
}
}
#endif