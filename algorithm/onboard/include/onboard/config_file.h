#ifndef _ONBOARD_CONFIG_FILE_H_
#define _ONBOARD_CONFIG_FILE_H_

// 板载系统的全部可配参数（对应 config/onboard.yaml）。
// 代码里的结构体默认值只是兜底，实机一律以 YAML 为准。

#include <string>
#include <vector>

#include <Eigen/Eigen>

#include "onboard/ego_planner_adapter.h"
#include "onboard/offboard_pilot.h"
#include "onboard/mission_input.h"
#include "onboard/livox_source.h"

namespace ego_onboard
{

  struct AppConfig
  {
    OffboardPilot::Config pilot;
    MissionInputConfig mission_input;
    bool extrinsic_confirmed = false;
    LivoxSource::Config lidar;
    EgoPlannerAdapterConfig adapter;

    std::string uplink;             // "ip:port"，空 = 不上报
    // 航线来自启动时解析的本地 KML。

    // 机体包络半径（用于校验膨胀量）
    double vehicle_radius = 0.85;
    double min_obstacle_gap = 1.5; // 含桨叶包络外的设计余量，不能只覆盖机体半径
  };

  // 读 YAML 并填充 out；缺省的键保持结构体默认值。
  // 返回 false 时 err 里是具体原因（文件打不开 / 语法错 / 参数不合理）。
  bool loadConfigFile(const std::string &path, AppConfig &out, std::string &err);

} // namespace ego_onboard

#endif
