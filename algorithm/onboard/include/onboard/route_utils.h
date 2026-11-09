#ifndef ONBOARD_ROUTE_UTILS_H
#define ONBOARD_ROUTE_UTILS_H

#include <cmath>
#include <string>
#include "onboard/offboard_pilot.h"

namespace ego_onboard
{
  // First KML coordinate is the agreed takeoff origin. Return is an ordinary
  // NAV leg at cruise height, never a firmware RTL / land command.
  inline bool ensureReturnToStart(std::vector<GeoWaypoint> &route, std::size_t max_points,
                                 bool &added, std::string &error)
  {
    added = false;
    if (route.size() < 2 || route.size() > max_points)
    { error = "Route must contain an origin and at least one task waypoint"; return false; }
    const auto &a = route.front();
    const auto &b = route.back();
    if (!std::isfinite(a.lat_deg) || !std::isfinite(a.lon_deg) || !std::isfinite(a.alt_rel_m) ||
        !std::isfinite(b.lat_deg) || !std::isfinite(b.lon_deg) || !std::isfinite(b.alt_rel_m) ||
        std::abs(a.lat_deg) > 90 || std::abs(b.lat_deg) > 90 ||
        std::abs(a.lon_deg) > 180 || std::abs(b.lon_deg) > 180)
    { error = "Invalid route endpoints"; return false; }
    // Compare at the precision actually sent by MISSION_ITEM_INT (degE7).
    const bool same = std::llround(a.lat_deg * 1e7) == std::llround(b.lat_deg * 1e7) &&
                      std::llround(a.lon_deg * 1e7) == std::llround(b.lon_deg * 1e7) &&
                      std::abs(a.alt_rel_m - b.alt_rel_m) < 0.01;
    if (same) route.back() = route.front();
    else
    {
      if (route.size() >= max_points)
      { error = "No waypoint capacity left for return to origin"; return false; }
      const GeoWaypoint origin = route.front();
      route.push_back(origin);
      added = true;
    }
    return true;
  }
}
#endif
