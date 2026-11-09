#ifndef ONBOARD_MISSION_INPUT_H
#define ONBOARD_MISSION_INPUT_H

#include <cstddef>
#include <string>
#include <vector>

#include "onboard/offboard_pilot.h"

namespace ego_onboard
{
  // One local KML task per run. Optional inbox waiting precedes device opening.
  // Cloud transport writes the configured path atomically; READY latches the task.
  struct MissionInputConfig
  {
    std::string file;
    bool wait_for_file = false;
    double file_settle_s = 0.5;
    std::size_t max_bytes = 5 * 1024 * 1024;
    std::size_t max_waypoints = 2000;    // includes the first takeoff point
    std::string path_name;              // select a LineString Placemark by name
    std::string height_mode = "constant"; // project default: fixed 50 m above HOME
    double home_amsl_m = 0.0;
    bool have_home_amsl = false;
    double constant_alt_m = 50.0;
    double min_alt_m = 0.5;             // all relative heights must exceed this
    double max_alt_m = 1000.0;
  };

  struct PreparedMission
  {
    std::vector<GeoWaypoint> route;     // route[0] = takeoff, route[1..] = NAV
    std::vector<std::string> warnings;
    std::string height_source;
    bool return_leg_added = false;
  };

  enum class MissionInputState
  {
    IDLE, WAIT_KML, LOADING_KML, PARSING_KML, VALIDATING_ROUTE, READY, FAILED
  };
  const char *missionInputStateName(MissionInputState state);

  // Strict XML parser for local KML input. It does not fetch
  // URLs, load external entities, or guess a terrain-to-HOME height offset.
  bool parseMissionKml(const std::string &xml, const MissionInputConfig &config,
                       PreparedMission &mission, std::string &error);
  bool validateMissionRoute(const PreparedMission &mission,
                            const MissionInputConfig &config, std::string &error);

  class MissionInput
  {
  public:
    void start(const MissionInputConfig &config);
    void step();
    void step(double monotonic_now); // Explicit clock for deterministic inbox tests.
    MissionInputState state() const { return state_; }
    const PreparedMission &mission() const { return mission_; }
    const std::string &error() const { return error_; }
  private:
    MissionInputConfig config_;
    MissionInputState state_ = MissionInputState::IDLE;
    PreparedMission mission_;
    std::string body_, error_;
    std::string observed_version_, rejected_version_;
    double stable_since_ = 0;
    void rejectOrFail();
    bool versionUnchanged();
  };
}
#endif
