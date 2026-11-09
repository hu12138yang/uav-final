#include <onboard/mission_input.h>
#include "fake_fcu.h"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>

using namespace ego_onboard;
using namespace ego_mav;

namespace
{
  int failures = 0;
  void check(bool ok, const char *name)
  {
    std::printf("[%s] %s\n", ok ? "OK" : "FAIL", name);
    if (!ok) ++failures;
  }
  const std::string kml_open = "<kml xmlns='http://www.opengis.net/kml/2.2'><Document>";
  const std::string kml_close = "</Document></kml>";
  std::string line(const std::string &coords, const std::string &name = "route")
  {
    return "<Placemark><name>" + name + "</name><LineString><altitudeMode>absolute</altitudeMode><coordinates>" +
           coords + "</coordinates></LineString></Placemark>";
  }
  bool ready(const std::string &xml, MissionInputConfig config = {})
  {
    PreparedMission mission;
    std::string error;
    return parseMissionKml(xml, config, mission, error) && validateMissionRoute(mission, config, error);
  }
}

int main(int argc, char **argv)
{
  if (argc != 2) { std::fprintf(stderr, "usage: mission_input_selftest sample.kml\n"); return 2; }
  MissionInputConfig config;
  config.file = argv[1];
  MissionInput input;
  input.start(config);
  check(input.state() == MissionInputState::LOADING_KML, "start enters local KML loading state");
  input.step();
  check(input.state() == MissionInputState::PARSING_KML, "sample file loads");
  input.step();
  check(input.state() == MissionInputState::VALIDATING_ROUTE, "sample XML parses");
  input.step();
  check(input.state() == MissionInputState::READY, "sample route validates");
  if (input.state() != MissionInputState::READY)
  { std::fprintf(stderr, "%s\n", input.error().c_str()); return 1; }

  const auto route = input.mission().route;
  check(route.size() == 6, "provided GCS KML has six valid route coordinates");
  check(input.mission().warnings.size() == 3, "three GCS nan,nan placeholders are reported");
  bool heights_ok = true;
  for (const auto &w : route) heights_ok &= std::abs(w.alt_rel_m - 50.0) < 1e-9;
  check(!input.mission().return_leg_added, "already closed sample does not get a duplicate return leg");
  check(heights_ok, "all heights are fixed to 50 m, ignoring KML AMSL=211 / Alt Rel=100");
  check(std::abs(route.front().lat_deg - 34.8193162) < 1e-9 &&
        std::abs(route.front().lon_deg - 113.4752016) < 1e-9,
        "KML lon,lat order is converted correctly");
  check(route.front().lat_deg == route.back().lat_deg && route.front().lon_deg == route.back().lon_deg,
        "closing waypoint is preserved (no silent loop truncation)");

  PreparedMission opened;
  std::string close_error;
  check(parseMissionKml(kml_open + line("113,21,0 113.001,21,0") + kml_close, config, opened, close_error) &&
        opened.return_leg_added && opened.route.size() == 3 &&
        opened.route.back().lat_deg == opened.route.front().lat_deg &&
        opened.route.back().lon_deg == opened.route.front().lon_deg && opened.route.back().alt_rel_m == 50,
        "open KML gains exactly one ordinary return waypoint at 50 m");

  check(ready(kml_open + line("113,21,211 113.0002,21,212") + kml_close), "LineString supported");
  check(ready("<k:kml xmlns:k='http://www.opengis.net/kml/2.2'><k:Placemark><k:LineString>"
              "<k:coordinates>113,21 113.001,21</k:coordinates></k:LineString></k:Placemark></k:kml>"),
        "namespace prefixes and missing altitude supported at fixed height");
  check(ready(kml_open + "<Placemark><Point><coordinates>113,21,0</coordinates></Point></Placemark>"
                     "<Placemark><Point><coordinates>113.001,21,0</coordinates></Point></Placemark>" + kml_close),
        "ordered Point-only KML supported");
  check(!ready(kml_open + line("bad,21,50 113,21,50") + kml_close), "non-numeric coordinate rejected");
  check(!ready(kml_open + line("181,21,50 113,21,50") + kml_close), "out-of-range longitude rejected");
  check(!ready(kml_open + line("nan,21,50 113,21,50") + kml_close), "invalid route vertex is not silently skipped");
  check(!ready(kml_open + line("113,21,50") + kml_close), "route with no NAV waypoint rejected");
  check(!ready("<html>login page</html>"), "non-KML payload rejected");
  check(!ready(kml_open + line("113,21,50 113.001,21,50")), "malformed XML rejected");
  check(!ready("<!DOCTYPE kml [<!ENTITY x SYSTEM 'file:///etc/passwd'>]><kml>&x;</kml>"),
        "DTD and external entities rejected");
  check(!ready(kml_open + line("113,21,0 113.001,21,0", "a") + line("113,22,0 113.001,22,0", "b") + kml_close),
        "multiple LineStrings require explicit selection");
  auto selected = config;
  selected.path_name = "b";
  check(ready(kml_open + line("113,21,0 113.001,21,0", "a") + line("113,22,0 113.001,22,0", "b") + kml_close, selected),
        "named path selection supported");
  auto zero = config;
  zero.constant_alt_m = 0;
  check(!ready(kml_open + line("113,21,50 113.001,21,50") + kml_close, zero), "zero flight height rejected");
  auto limited = config;
  limited.max_waypoints = 2;
  check(!ready(kml_open + line("113,21,0 113.001,21,0 113.002,21,0") + kml_close, limited), "waypoint limit enforced");
  limited = config;
  limited.max_bytes = 20;
  check(!ready(kml_open + line("113,21,0 113.001,21,0") + kml_close, limited), "file size limit enforced");
  auto missing = config;
  missing.file = "/nonexistent/uav-localV2/mission.kml";
  input.start(missing);
  input.step();
  check(input.state() == MissionInputState::FAILED && input.mission().route.empty(),
        "failed new file clears the previous task and cannot reuse it");

  // Feed the REAL prepared route through V2's real upload state machine,
  // real encoder/decoder and the existing fake FCU. Stop at upload ACK;
  // no physical serial port is opened and no real aircraft is commanded.
  ego_test::FakeFcu fcu;
  fcu.lat0 = route.front().lat_deg;
  fcu.lon0 = route.front().lon_deg;
  fcu.route = route;
  fcu.takeoff_alt = 50;
  OffboardPilot pilot;
  OffboardPilot::Config pilot_config;
  pilot_config.cruise_alt_m = route.front().alt_rel_m;
  pilot.setConfig(pilot_config);
  pilot.start(route);
  for (double now = 0; now < 5 && pilot.phase() != Phase::WAIT_GROUND_START && pilot.phase() != Phase::FAILED; now += 0.01)
  {
    std::vector<std::vector<uint8_t>> downlink;
    fcu.tick(0.01, now, downlink);
    for (const auto &f : downlink) pilot.onMavlinkBytes(f.data(), f.size());
    std::vector<std::vector<uint8_t>> uplink, response;
    pilot.tick(now, uplink);
    for (const auto &f : uplink) fcu.onBytes(f.data(), f.size(), response);
    for (const auto &f : response) pilot.onMavlinkBytes(f.data(), f.size());
  }
  check(pilot.phase() == Phase::WAIT_GROUND_START, "KML -> actual V2 upload -> FCU ACK completes");
  check(fcu.recv_count44 == 1 && fcu.mission.size() == route.size(),
        "one MISSION_COUNT and all six mission items uploaded");
  if (fcu.mission.size() == route.size())
  {
    check(fcu.mission.front().command == 178 && fcu.mission.back().command == 16 &&
          fcu.mission.back().z == 50 && fcu.mission.back().frame == 3,
          "speed + NAV only, last NAV returns to origin at 50 m");
    bool equal = true;
    for (std::size_t i = 1; i < route.size(); ++i)
    {
      const auto &it = fcu.mission[i];
      equal &= it.command == 16 && it.frame == 3 && it.seq == i && it.z == 50 &&
               it.x == static_cast<int32_t>(std::llround(route[i].lat_deg * 1e7)) &&
               it.y == static_cast<int32_t>(std::llround(route[i].lon_deg * 1e7));
    }
    check(equal, "every uploaded NAV coordinate, height and order equals parsed KML");
  }
  check(!fcu.armed && fcu.received_commands.empty(), "upload ACK does not arm or activate AUTO");
  check(pilot.crcErrors() == 0 && fcu.parser.crcErrors() == 0, "upload CRC valid in both directions");
  std::printf("mission_input_selftest: %s (fail=%d)\n", failures ? "FAILED" : "PASSED", failures);
  return failures ? 1 : 0;
}
