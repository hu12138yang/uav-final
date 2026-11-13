#include "onboard/config_file.h"

#include <cmath>
#include <sstream>

#include <yaml-cpp/yaml.h>

namespace ego_onboard
{
  namespace
  {
    // 键不存在就保持原值（结构体默认值）
    template <typename T>
    void get(const YAML::Node &n, const char *key, T &dst)
    {
      if (n && n[key])
        dst = n[key].as<T>();
    }

    // uint8_t 会被 yaml-cpp 当成字符类型，必须先读 int 再收窄
    void getU8(const YAML::Node &n, const char *key, uint8_t &dst)
    {
      if (n && n[key])
        dst = (uint8_t)n[key].as<int>();
    }

    Eigen::Vector3d vec3(const YAML::Node &n)
    {
      Eigen::Vector3d v = Eigen::Vector3d::Zero();
      if (!n)
        return v;
      const auto a = n.as<std::vector<double>>();
      if (a.size() >= 3)
        v = Eigen::Vector3d(a[0], a[1], a[2]);
      return v;
    }

    Eigen::Vector2d vec2(const YAML::Node &n)
    {
      Eigen::Vector2d v = Eigen::Vector2d::Zero();
      if (!n)
        return v;
      const auto a = n.as<std::vector<double>>();
      if (a.size() >= 2)
        v = Eigen::Vector2d(a[0], a[1]);
      return v;
    }

  } // namespace

  bool loadConfigFile(const std::string &path, AppConfig &out, std::string &err)
  {
    YAML::Node root;
    try
    {
      root = YAML::LoadFile(path);
    }
    catch (const std::exception &e)
    {
      err = std::string("读取 ") + path + " 失败: " + e.what();
      return false;
    }

    try {
    if (const auto n = root["mission_input"]) {
      get(n,"file",out.mission_input.file); get(n,"path_name",out.mission_input.path_name);
      get(n,"wait_for_file",out.mission_input.wait_for_file); get(n,"file_settle_s",out.mission_input.file_settle_s);
      get(n,"max_bytes",out.mission_input.max_bytes); get(n,"max_waypoints",out.mission_input.max_waypoints);
      get(n,"height_mode",out.mission_input.height_mode); get(n,"constant_alt_m",out.mission_input.constant_alt_m);
    }
    if (const auto n=root["lidar"]) {
      std::string policy="strict";get(n,"crc_policy",policy);
      if(policy!="strict" && policy!="mid360s_sample") throw std::runtime_error("lidar.crc_policy must be strict or mid360s_sample");
      out.lidar.allow_zero_crc=policy=="mid360s_sample";
      get(n,"expected_serial",out.lidar.expected_serial);get(n,"point_data_type",out.lidar.point_data_type);
      get(n,"frame_duration_s",out.lidar.frame_duration_s);get(n,"gap_factor",out.lidar.gap_factor);
      get(n,"min_range_m",out.lidar.min_range_m);get(n,"max_range_m",out.lidar.max_range_m);
      get(n,"queue_capacity",out.lidar.queue_capacity);
      if(!std::isfinite(out.lidar.frame_duration_s)||out.lidar.frame_duration_s<0.02||out.lidar.frame_duration_s>0.2 ||
         !std::isfinite(out.lidar.gap_factor)||out.lidar.gap_factor<1.0||out.lidar.gap_factor>4.0 ||
         !std::isfinite(out.lidar.min_range_m)||!std::isfinite(out.lidar.max_range_m)||out.lidar.min_range_m<0||out.lidar.max_range_m<=out.lidar.min_range_m ||
         out.lidar.queue_capacity<1||out.lidar.queue_capacity>10||out.lidar.point_data_type!=1 ||
         (out.lidar.allow_zero_crc&&out.lidar.expected_serial.empty())) throw std::runtime_error("invalid measured lidar profile");
    }
    // ---------------- 飞控协议身份与链路 ----------------
    if (const YAML::Node n = root["mavlink"])
    {
      getU8(n, "sysid", out.pilot.sysid);
      getU8(n, "compid", out.pilot.compid);
      getU8(n, "target_system", out.pilot.target_system);
      getU8(n, "target_component", out.pilot.target_component);
      get(n, "heartbeat_hz", out.pilot.heartbeat_hz);
      get(n, "link_timeout", out.pilot.link_timeout);
      get(n, "command_ack_timeout", out.pilot.command_ack_timeout);
      get(n, "max_retry", out.pilot.max_retry);
    }

    // ---------------- 任务 ----------------
    if (const YAML::Node n = root["mission"])
    {
      get(n, "cruise_speed", out.pilot.cruise_speed);
      get(n, "handover_alt_tolerance_m", out.pilot.handover_alt_tolerance_m);
      get(n, "handover_position_tolerance_m", out.pilot.handover_position_tolerance_m);
      get(n, "wp_reach_tol", out.pilot.wp_reach_tol);

      get(n, "pre_offboard_stream_s", out.pilot.pre_offboard_stream_s);
      get(n, "use_planner", out.pilot.use_planner);
      // 航线来自启动时解析的本地 KML。
    }

    // ---------------- 报文提频（协议 2.5.1） ----------------
    if (const YAML::Node n = root["streaming"])
    {
      get(n, "request_rates", out.pilot.request_rates);
      get(n, "global_position_hz", out.pilot.rate_global_position_hz);
      get(n, "attitude_hz", out.pilot.rate_attitude_hz);
      get(n, "extension_hz", out.pilot.rate_extension_hz);
    }

    if (const auto n = root["timing"]) {
      get(n, "position_extrapolation_s", out.pilot.position_extrapolation_s);
    }

    // ---------------- 异常处理 / 悬停 ----------------
    if (const YAML::Node n = root["failsafe"])
    {
      if (const YAML::Node g = n["gps"])
      {
        get(g, "enabled", out.pilot.gps_failsafe);
        getU8(g, "min_fix", out.pilot.gps_min_fix);
        get(g, "loss_confirm_s", out.pilot.gps_loss_confirm_s);
        get(g, "rx_timeout_s", out.pilot.gps_rx_timeout_s);
      }
      if (const YAML::Node s = n["sensing"])
      {
        get(s, "enabled", out.pilot.sensing_failsafe);
        get(s, "drop_frames", out.adapter.sensing_drop_frames);
        get(s, "timeout_s", out.adapter.sensing_timeout_s);
      }
      get(n, "hold_recover_confirm_s", out.pilot.hold_recover_confirm_s);
    }

    // ---------------- 输出形态 ----------------
    if (const YAML::Node n = root["output"]) {
      get(n, "velocity_feedforward", out.pilot.velocity_feedforward);
      get(n, "setpoint_hz", out.pilot.setpoint_hz);
    }

    // ---------------- 机体 ----------------
    if (const YAML::Node n = root["vehicle"]) {
      get(n, "radius", out.vehicle_radius);
      get(n, "min_obstacle_gap", out.min_obstacle_gap);
    }

    // ---------------- 地图 ----------------
    if (const YAML::Node n = root["map"])
    {
      get(n, "resolution", out.adapter.resolution);
      if (n["size_xy"])
        out.adapter.map_size_xy = vec2(n["size_xy"]);
      get(n, "altitude_band_half", out.adapter.altitude_band_half);
      get(n, "max_descent_below_ref", out.adapter.max_descent_below_ref);
      get(n, "map_margin_z", out.adapter.map_margin_z);
      get(n, "min_flight_height", out.adapter.min_flight_height);
      get(n, "update_range_xy", out.adapter.occupancy_update_range_xy);
      get(n, "update_range_z", out.adapter.occupancy_update_range_z);
      get(n, "obstacles_inflation", out.adapter.obstacles_inflation);
      get(n, "local_map_margin", out.adapter.local_map_margin);
      get(n, "recenter_margin", out.adapter.recenter_margin);
    }

    // ---------------- 规划器 ----------------
    if (const YAML::Node n = root["planner"])
    {
      get(n, "max_vel", out.adapter.max_vel);
      get(n, "max_acc", out.adapter.max_acc);
      get(n, "planning_horizon", out.adapter.planning_horizon);
      get(n, "local_target_step", out.adapter.local_target_step);
      get(n, "local_target_max_ratio", out.adapter.local_target_max_ratio);
      get(n, "replan_period", out.adapter.replan_period);
      get(n, "goal_tolerance", out.adapter.goal_tolerance);
      get(n, "obstacle_clearance", out.adapter.obstacle_clearance);
      get(n, "ctrl_pt_dist", out.adapter.ctrl_pt_dist);
      get(n, "time_forward", out.adapter.time_forward);
      if (const YAML::Node p = n["astar_pool"])
      {
        const auto a = p.as<std::vector<int>>();
        if (a.size() == 3)
          out.adapter.astar_pool_size = Eigen::Vector3i(a[0], a[1], a[2]);
      }
    }

    // ---------------- 外参：雷达 -> 机体 ----------------
    if (const YAML::Node n = root["extrinsic"])
    {
      get(n,"confirmed",out.extrinsic_confirmed);
      if (n["R"])
      {
        const auto a = n["R"].as<std::vector<double>>();
        if (a.size() == 9)
        {
          Eigen::Matrix3d R;
          for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
              R(r, c) = a[r * 3 + c];
          out.adapter.extrinsic.R = R;
        }
        else
        {
          err = "extrinsic.R 需要 9 个数（行优先）";
          return false;
        }
      }
      if (n["t"])
        out.adapter.extrinsic.t = vec3(n["t"]);
    }

    // ---------------- 上报链路 ----------------
    if (root["uplink"])
      out.uplink = root["uplink"].as<std::string>();

    // ---------------- 参数校验 ----------------
    if (!std::isfinite(out.pilot.position_extrapolation_s) ||
        out.pilot.position_extrapolation_s <= 0 || out.pilot.position_extrapolation_s > 2.0) {
      err = "timing.position_extrapolation_s must be finite and in (0, 2] seconds";
      return false;
    }
    // 显式覆盖含桨叶的机体包络及所需桨尖余量；不把优化器软约束当硬间距。
    if (!std::isfinite(out.vehicle_radius) || out.vehicle_radius <= 0 ||
        !std::isfinite(out.min_obstacle_gap) || out.min_obstacle_gap < 0 ||
        !std::isfinite(out.adapter.obstacles_inflation)) {
      err = "vehicle radius, min_obstacle_gap and obstacles_inflation must be finite and valid";
      return false;
    }
    if (out.adapter.obstacles_inflation + 1e-9 < out.vehicle_radius + out.min_obstacle_gap)
    {
      std::ostringstream os;
      os << "obstacles_inflation (" << out.adapter.obstacles_inflation
         << " m) 小于机体半径 (" << out.vehicle_radius << " m) + 桨尖余量 ("
         << out.min_obstacle_gap << " m)";
      err = os.str();
      return false;
    }
    if (out.pilot.wp_reach_tol <= 0.0)
    {
      err = "mission.wp_reach_tol 必须 > 0";
      return false;
    }
    if (out.adapter.resolution <= 0.0)
    {
      err = "map.resolution 必须 > 0";
      return false;
    }

    // 滚动窗口必须完整容纳入图范围：机体最多偏离窗口中心 recenter_margin，
    // 此时前方只剩 半宽 - recenter_margin。不够时远端的点被静默丢弃，
    // 而 A* 把图外（-1）当成占用，等于凭空多出一堵墙。
    const double half_x = 0.5 * out.adapter.map_size_xy.x();
    const double half_y = 0.5 * out.adapter.map_size_xy.y();
    const double half_xy = (half_x < half_y) ? half_x : half_y;
    if (half_xy + 1e-9 < out.adapter.recenter_margin + out.adapter.occupancy_update_range_xy)
    {
      std::ostringstream os;
      os << "map.size_xy 半宽 (" << half_xy << " m) 小于 recenter_margin ("
         << out.adapter.recenter_margin << " m) + update_range_xy ("
         << out.adapter.occupancy_update_range_xy << " m)，滚动到边界时会丢点";
      err = os.str();
      return false;
    }

    // 高度带必须被垂直入图范围盖住，否则"向上抬升避障"的目标点会落进盲区
    if (out.adapter.occupancy_update_range_z + 1e-9 < out.adapter.altitude_band_half)
    {
      std::ostringstream os;
      os << "map.update_range_z (" << out.adapter.occupancy_update_range_z
         << " m) 小于 altitude_band_half (" << out.adapter.altitude_band_half
         << " m)，高度带边缘不会有数据";
      err = os.str();
      return false;
    }
    if (out.adapter.max_descent_below_ref < 0.0)
    {
      err = "map.max_descent_below_ref 必须 >= 0";
      return false;
    }

    if (!(out.pilot.setpoint_hz >= 5 && out.pilot.setpoint_hz <= 50) ||
        !(out.adapter.sensing_timeout_s > 0 && out.adapter.sensing_timeout_s <= 1.0) ||
        !out.adapter.extrinsic.R.allFinite() || !out.adapter.extrinsic.t.allFinite() ||
        (out.adapter.extrinsic.R.transpose()*out.adapter.extrinsic.R-Eigen::Matrix3d::Identity()).norm()>1e-5 ||
        std::abs(out.adapter.extrinsic.R.determinant()-1.0)>1e-5) {
      err="invalid output rate, perception timeout, or rigid extrinsic rotation"; return false;
    }
    return true;
    } catch (const std::exception &e) { err=std::string("invalid config: ")+e.what(); return false; }
  }

} // namespace ego_onboard
