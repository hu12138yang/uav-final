#include "onboard/mission_input.h"
#include "onboard/route_utils.h"

#include <libxml/parser.h>
#include <libxml/tree.h>

#include <algorithm>
#include <cerrno>
#include <climits>
#include <cmath>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <sstream>
#include <sys/stat.h>

namespace ego_onboard
{
  namespace
  {
    struct Sample
    {
      double lon = 0, lat = 0, alt = 0, relative = 0;
      bool have_alt = false, have_relative = false;
      std::string altitude_mode;
    };
    struct Path { std::string name; std::vector<Sample> points; };

    bool named(xmlNode *node, const char *name)
    {
      if (!node || node->type != XML_ELEMENT_NODE || !xmlStrEqual(node->name, BAD_CAST name))
        return false;
      return !node->ns || !node->ns->href ||
             xmlStrEqual(node->ns->href, BAD_CAST "http://www.opengis.net/kml/2.2");
    }
    std::string content(xmlNode *node)
    {
      if (!node) return {};
      xmlChar *value = xmlNodeGetContent(node);
      std::string result = value ? reinterpret_cast<const char *>(value) : "";
      xmlFree(value);
      return result;
    }
    xmlNode *child(xmlNode *node, const char *name)
    {
      for (xmlNode *c = node ? node->children : nullptr; c; c = c->next)
        if (named(c, name)) return c;
      return nullptr;
    }
    std::string trim(const std::string &s)
    {
      const auto a = s.find_first_not_of(" \t\r\n");
      if (a == std::string::npos) return {};
      return s.substr(a, s.find_last_not_of(" \t\r\n") - a + 1);
    }
    bool number(const std::string &text, double &value, bool allow_nan = false)
    {
      const std::string s = trim(text);
      if (s.empty()) return false;
      errno = 0;
      char *end = nullptr;
      value = std::strtod(s.c_str(), &end);
      return end == s.c_str() + s.size() && errno != ERANGE &&
             (std::isfinite(value) || (allow_nan && std::isnan(value)));
    }
    bool relativeDescription(const std::string &description, double &height)
    {
      const auto pos = description.find("Alt Rel:");
      if (pos == std::string::npos) return false;
      std::istringstream stream(description.substr(pos + 8));
      std::string token;
      stream >> token;
      return number(token, height);
    }
    bool coordinates(xmlNode *geometry, bool point, std::size_t limit, std::vector<Sample> &result,
                     std::vector<std::string> &warnings, std::string &error)
    {
      xmlNode *coordinate_node = child(geometry, "coordinates");
      if (!coordinate_node) { error = "KML geometry has no coordinates"; return false; }
      const std::string mode = trim(content(child(geometry, "altitudeMode")));
      std::istringstream input(content(coordinate_node));
      std::string token;
      std::size_t tuples = 0;
      while (input >> token)
      {
        ++tuples;
        if (point && tuples > 1) { error = "Point must contain exactly one coordinate"; return false; }
        std::vector<std::string> fields;
        std::size_t offset = 0;
        for (;;)
        {
          auto comma = token.find(',', offset);
          fields.push_back(token.substr(offset, comma == std::string::npos ? comma : comma - offset));
          if (comma == std::string::npos) break;
          offset = comma + 1;
        }
        Sample sample;
        if (fields.size() < 2 || fields.size() > 3 ||
            !number(fields[0], sample.lon, true) || !number(fields[1], sample.lat, true))
        { error = "Invalid KML coordinate tuple"; return false; }
        // GCS PRO emits nan,nan,0 Points for non-positional task placeholders.
        // A malformed route vertex / a partly invalid Point is never skipped.
        if (point && std::isnan(sample.lon) && std::isnan(sample.lat))
        {
          warnings.push_back("Skipped GCS Point placeholder with nan,nan coordinates");
          continue;
        }
        if (!std::isfinite(sample.lon) || !std::isfinite(sample.lat) ||
            std::abs(sample.lon) > 180 || std::abs(sample.lat) > 90)
        { error = "KML longitude/latitude is non-finite or outside its valid range"; return false; }
        sample.have_alt = fields.size() == 3 && number(fields[2], sample.alt);
        if (fields.size() == 3 && !sample.have_alt)
        { error = "KML coordinate altitude is invalid"; return false; }
        sample.altitude_mode = mode;
        if (result.size() >= limit) { error = "KML has too many coordinates"; return false; }
        result.push_back(sample);
      }
      if (tuples == 0) { error = "KML coordinates are empty"; return false; }
      return true;
    }
    bool collect(xmlNode *node, std::size_t limit, std::vector<Sample> &points, std::vector<Path> &paths,
                 std::vector<std::string> &warnings, std::string &error)
    {
      for (; node; node = node->next)
      {
        if (named(node, "Placemark"))
        {
          xmlNode *point = child(node, "Point"), *line = child(node, "LineString");
          if (point && line) { error = "Ambiguous Placemark contains both Point and LineString"; return false; }
          if (point)
          {
            std::vector<Sample> values;
            if (!coordinates(point, true, limit, values, warnings, error)) return false;
            if (!values.empty())
            {
              values[0].have_relative = relativeDescription(content(child(node, "description")), values[0].relative);
              if (points.size() >= limit) { error = "KML has too many Point waypoints"; return false; }
              points.push_back(values[0]);
            }
          }
          else if (line)
          {
            Path path;
            path.name = trim(content(child(node, "name")));
            if (!coordinates(line, false, limit, path.points, warnings, error)) return false;
            if (paths.size() >= limit) { error = "KML has too many paths"; return false; }
            paths.push_back(std::move(path));
          }
          else if (child(node, "MultiGeometry"))
          { error = "MultiGeometry mission is unsupported; export a single LineString or ordered Points"; return false; }
        }
        if (warnings.size() > limit) { error = "KML has too many placeholder Points"; return false; }
        if (!collect(node->children, limit, points, paths, warnings, error)) return false;
      }
      return true;
    }
    bool configValid(const MissionInputConfig &c, std::string &error)
    {
      if (!std::isfinite(c.file_settle_s) || c.file_settle_s < 0.1 || c.file_settle_s > 30)
      { error = "mission_input.file_settle_s must be 0.1..30 seconds"; return false; }
      if (c.max_bytes == 0 || c.max_bytes > static_cast<std::size_t>(INT_MAX) ||
          c.max_waypoints < 2 || c.max_waypoints > 65533 ||
          !std::isfinite(c.min_alt_m) || !std::isfinite(c.max_alt_m) ||
          c.min_alt_m < 0 || c.max_alt_m <= c.min_alt_m)
      { error = "Invalid mission_input limits"; return false; }
      if (c.height_mode != "auto" && c.height_mode != "relative" &&
          c.height_mode != "absolute" && c.height_mode != "constant")
      { error = "height_mode must be auto, relative, absolute, or constant"; return false; }
      if ((c.have_home_amsl && !std::isfinite(c.home_amsl_m)) ||
          (c.height_mode == "constant" && !std::isfinite(c.constant_alt_m)))
      { error = "Invalid configured altitude"; return false; }
      return true;
    }

    bool loadFile(const MissionInputConfig &c, std::string &body, std::string &error)
    {
      std::ifstream file(c.file, std::ios::binary);
      if (!file) { error = "Cannot open local KML: " + c.file; return false; }
      char chunk[8192];
      body.clear();
      while (file.read(chunk, sizeof(chunk)) || file.gcount())
      {
        const auto n = static_cast<std::size_t>(file.gcount());
        if (n > c.max_bytes - body.size()) { error = "KML exceeds max_bytes"; return false; }
        body.append(chunk, n);
      }
      if (!file.eof()) { error = "Local KML read failed"; return false; }
      return true;
    }

    bool fileVersion(const std::string &path, std::string &version, std::string &error)
    {
      struct stat info;
      if (::stat(path.c_str(), &info) != 0)
      {
        error = errno == ENOENT ? "Waiting for local KML: " + path : "Cannot stat local KML: " + path;
        return false;
      }
      if (!S_ISREG(info.st_mode))
      { error = "KML inbox must be a regular file: " + path; return false; }
      std::ostringstream out;
      out << info.st_dev << ':' << info.st_ino << ':' << info.st_size << ':'
          << info.st_mtim.tv_sec << ':' << info.st_mtim.tv_nsec << ':'
          << info.st_ctim.tv_sec << ':' << info.st_ctim.tv_nsec;
      version = out.str();
      return true;
    }

  }

  const char *missionInputStateName(MissionInputState state)
  {
    switch (state)
    {
    case MissionInputState::IDLE: return "IDLE";
    case MissionInputState::WAIT_KML: return "WAIT_KML";
    case MissionInputState::LOADING_KML: return "LOADING_KML";
    case MissionInputState::PARSING_KML: return "PARSING_KML";
    case MissionInputState::VALIDATING_ROUTE: return "VALIDATING_ROUTE";
    case MissionInputState::READY: return "READY";
    case MissionInputState::FAILED: return "FAILED";
    }
    return "UNKNOWN";
  }

  bool parseMissionKml(const std::string &xml, const MissionInputConfig &config,
                       PreparedMission &mission, std::string &error)
  {
    mission = PreparedMission{};
    error.clear();
    if (!configValid(config, error)) return false;
    if (xml.empty() || xml.size() > config.max_bytes)
    { error = "KML is empty or exceeds max_bytes"; return false; }
    std::unique_ptr<xmlDoc, decltype(&xmlFreeDoc)> doc(
        xmlReadMemory(xml.data(), static_cast<int>(xml.size()), "mission.kml", nullptr,
                      XML_PARSE_NONET | XML_PARSE_NOBLANKS | XML_PARSE_NOERROR | XML_PARSE_NOWARNING), xmlFreeDoc);
    if (!doc || doc->intSubset || doc->extSubset || !named(xmlDocGetRootElement(doc.get()), "kml"))
    { error = "Expected valid KML XML without DTD / external entities"; return false; }
    std::vector<Sample> points;
    std::vector<Path> paths;
    if (!collect(xmlDocGetRootElement(doc.get()), config.max_waypoints, points, paths, mission.warnings, error)) return false;
    std::vector<Sample> chosen;
    if (!paths.empty())
    {
      if (config.path_name.empty())
      {
        if (paths.size() != 1) { error = "Multiple LineStrings: set mission_input.path_name"; return false; }
        chosen = paths.front().points;
      }
      else
      {
        int matches = 0;
        for (const auto &path : paths)
          if (path.name == config.path_name) { chosen = path.points; ++matches; }
        if (matches != 1) { error = "path_name must select exactly one LineString"; return false; }
      }
    }
    else
    {
      if (!config.path_name.empty()) { error = "path_name given but no LineString exists"; return false; }
      chosen = points;
    }
    if (chosen.empty() || chosen.size() > config.max_waypoints)
    { error = "KML has no route or too many waypoints"; return false; }
    for (auto &sample : chosen)
    {
      // A GCS flight-path LineString often contains AMSL coordinates, while
      // matching Point descriptions carry the intended relative altitude.
      for (const auto &point : points)
      {
        if (config.height_mode != "auto") break;
        if (!point.have_relative || std::abs(point.lon - sample.lon) > 1e-8 ||
            std::abs(point.lat - sample.lat) > 1e-8) continue;
        if (sample.have_relative && std::abs(sample.relative - point.relative) > 1e-6)
        { error = "Conflicting Alt Rel values for the same coordinate"; return false; }
        sample.relative = point.relative;
        sample.have_relative = true;
      }
      double height = 0;
      std::string source;
      if (config.height_mode == "constant")
      { height = config.constant_alt_m; source = "configured constant altitude"; }
      else if (config.height_mode == "relative")
      {
        if (!sample.have_alt) { error = "Relative mode requires coordinate altitude"; return false; }
        height = sample.alt; source = "coordinate altitude explicitly treated as relative to HOME";
      }
      else if (config.height_mode == "auto" && sample.have_relative)
      { height = sample.relative; source = "GCS Alt Rel"; }
      else if (sample.have_alt && config.have_home_amsl &&
               (config.height_mode == "absolute" || sample.altitude_mode == "absolute"))
      { height = sample.alt - config.home_amsl_m; source = "AMSL minus configured HOME AMSL"; }
      else
      {
        error = "Ambiguous KML altitude: supply Alt Rel, absolute HOME AMSL, or explicit relative/constant mode";
        return false;
      }
      if (mission.height_source.empty()) mission.height_source = source;
      else if (mission.height_source != source) mission.height_source = "mixed explicit altitude sources";
      mission.route.push_back({sample.lat, sample.lon, height});
    }
    return ensureReturnToStart(mission.route, config.max_waypoints, mission.return_leg_added, error);
  }

  bool validateMissionRoute(const PreparedMission &mission,
                            const MissionInputConfig &config, std::string &error)
  {
    if (!configValid(config, error)) return false;
    if (mission.route.size() < 2 || mission.route.size() > config.max_waypoints)
    { error = "Prepared route must have 2..max_waypoints points including takeoff"; return false; }
    for (const auto &w : mission.route)
      if (!std::isfinite(w.lat_deg) || !std::isfinite(w.lon_deg) || !std::isfinite(w.alt_rel_m) ||
          std::abs(w.lat_deg) > 90 || std::abs(w.lon_deg) > 180 ||
          w.alt_rel_m <= config.min_alt_m || w.alt_rel_m > config.max_alt_m)
      { error = "Route contains invalid coordinates or relative altitude outside configured limits"; return false; }
    return true;
  }

  void MissionInput::start(const MissionInputConfig &config)
  {
    config_ = config;
    mission_ = PreparedMission{};
    body_.clear(); error_.clear();
    observed_version_.clear(); rejected_version_.clear(); stable_since_ = 0;
    state_ = config.wait_for_file ? MissionInputState::WAIT_KML : MissionInputState::LOADING_KML;
    if (!configValid(config_, error_)) state_ = MissionInputState::FAILED;
    else if (config_.file.empty())
    {
      error_ = "mission_input.file is required";
      state_ = MissionInputState::FAILED;
    }
  }

  void MissionInput::step()
  {
    step(std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count());
  }

  void MissionInput::rejectOrFail()
  {
    mission_ = PreparedMission{};
    body_.clear();
    rejected_version_ = observed_version_;
    state_ = config_.wait_for_file ? MissionInputState::WAIT_KML : MissionInputState::FAILED;
  }

  bool MissionInput::versionUnchanged()
  {
    if (!config_.wait_for_file) return true;
    std::string version, error;
    if (fileVersion(config_.file, version, error) && version == observed_version_) return true;
    mission_ = PreparedMission{}; body_.clear(); observed_version_.clear();
    error_ = "KML changed while loading; waiting for a stable delivery";
    state_ = MissionInputState::WAIT_KML;
    return false;
  }

  void MissionInput::step(double now)
  {
    switch (state_)
    {
    case MissionInputState::WAIT_KML:
    {
      std::string version, error;
      if (!fileVersion(config_.file, version, error))
      { observed_version_.clear(); error_ = error; break; }
      if (version != observed_version_)
      { observed_version_ = version; stable_since_ = now; error_.clear(); break; }
      if (version == rejected_version_ || now - stable_since_ < config_.file_settle_s) break;
      error_.clear();
      state_ = MissionInputState::LOADING_KML;
      break;
    }
    case MissionInputState::LOADING_KML:
      if (!loadFile(config_, body_, error_)) rejectOrFail();
      else if (versionUnchanged()) state_ = MissionInputState::PARSING_KML;
      break;
    case MissionInputState::PARSING_KML:
      if (!parseMissionKml(body_, config_, mission_, error_)) rejectOrFail();
      else state_ = MissionInputState::VALIDATING_ROUTE;
      body_.clear();
      break;
    case MissionInputState::VALIDATING_ROUTE:
      if (!validateMissionRoute(mission_, config_, error_)) rejectOrFail();
      else if (versionUnchanged()) { state_ = MissionInputState::READY; error_.clear(); }
      break;
    default: break;
    }
  }
}
