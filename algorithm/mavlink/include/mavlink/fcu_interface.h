#ifndef _MAVLINK_FCU_INTERFACE_H_
#define _MAVLINK_FCU_INTERFACE_H_

// 语义层：把飞控 MAVLink 与我们的规划器对齐。
//
// 三处坐标系/约定差异（协议文档 2.2 节 + 2.4.13 节）：
//   1. ID=84 只支持 NED 系（coordinate_frame 选项里没有 ENU），我们规划是 ENU
//      → 位置/速度/加速度 + yaw/yaw_rate 都要转
//   2. ID=84 的 MAV_FRAME_LOCAL_NED 原点 = **起飞点(home)**，我们的 ENU 原点必须与它一致
//      → 用 HOME_POSITION(242) 取原点
//   3. ATTITUDE 是 NED/FRD，规划侧是 ENU/FLU → 姿态四元数要转

#include <Eigen/Eigen>
#include <cmath>
#include <algorithm>

#include <ego_core/trajectory_tracker.h> // QuadrotorCommand
#include <sensor/lidar_types.h>          // TimedPose / PoseBuffer

#include "mavlink/fcu_messages.h"

namespace ego_mav
{

  constexpr double kPi = 3.14159265358979323846;

  inline double wrapPi(double a)
  {
    while (a > kPi) a -= 2 * kPi;
    while (a < -kPi) a += 2 * kPi;
    return a;
  }

  // ------------------------------------------------------------- ENU <-> NED
  // MAV_FRAME_LOCAL_NED: x=北 y=东 z=下；规划系 ENU: x=东 y=北 z=上
  inline Eigen::Vector3d enuToNed(const Eigen::Vector3d &v) { return Eigen::Vector3d(v.y(), v.x(), -v.z()); }
  inline Eigen::Vector3d nedToEnu(const Eigen::Vector3d &v) { return Eigen::Vector3d(v.y(), v.x(), -v.z()); }

  // ENU 偏航（从 +x=东 起，逆时针）-> NED 偏航（从 +y=北 起，顺时针）
  inline double yawEnuToNed(double yaw_enu) { return wrapPi(kPi * 0.5 - yaw_enu); }
  inline double yawNedToEnu(double yaw_ned) { return wrapPi(kPi * 0.5 - yaw_ned); }
  // 偏航角速率方向相反
  inline double yawRateEnuToNed(double rate_enu) { return -rate_enu; }

  // ATTITUDE(NED/FRD) -> 规划侧用的 机体系(FLU) -> 世界(ENU)
  inline Eigen::Quaterniond attitudeNedToEnuBody(double roll, double pitch, double yaw)
  {
    const Eigen::Matrix3d R = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                               Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
                                  .toRotationMatrix();
    Eigen::Matrix3d C; // NED -> ENU
    C << 0, 1, 0,
         1, 0, 0,
         0, 0, -1;
    const Eigen::Matrix3d F = Eigen::Vector3d(1, -1, -1).asDiagonal(); // FRD -> FLU
    return Eigen::Quaterniond(C * R * F).normalized();
  }

  // ------------------------------------------------------- 地理原点（= home）
  struct GeoOrigin
  {
    double lat_deg = 0.0, lon_deg = 0.0; // 起飞点（返航点）
    bool valid = false;
    static constexpr double kR = 6378137.0; // 与 KmlReader::toENU 同一个等距圆柱近似
  };

  // 经纬 -> 局部 ENU 平面坐标（米）。高度不在这里算：
  // GLOBAL_POSITION_INT 直接给了 relative_alt（相对起飞点，mm），语义与"定高"一致。
  inline Eigen::Vector2d geoToEnuXY(double lat_deg, double lon_deg, const GeoOrigin &o)
  {
    const double lat0 = o.lat_deg * kPi / 180.0;
    const double x = (lon_deg - o.lon_deg) * kPi / 180.0 * std::cos(lat0) * GeoOrigin::kR; // 东
    const double y = (lat_deg - o.lat_deg) * kPi / 180.0 * GeoOrigin::kR;                 // 北
    return Eigen::Vector2d(x, y);
  }

  // ------------------------------------------------------------ 时间对齐
  // 飞控给的是 time_boot_ms（开机毫秒），规划器用自己的秒时钟。
  // 真实系统里这一步应由 PTP/PPS 硬件同步来做；这里给出最简可用的线性对齐。
  class ClockSync
  {
  public:
    void update(uint32_t t_boot_ms, double t_host_s)
    {
      if (!valid_)
      {
        offset_ = t_host_s - (double)t_boot_ms * 1e-3;
        valid_ = true;
        return;
      }
      // 缓慢跟随，避免单次抖动把整个时钟带偏
      const double measured = t_host_s - (double)t_boot_ms * 1e-3;
      offset_ += 0.01 * (measured - offset_);
    }
    bool valid() const { return valid_; }
    double hostTime(uint32_t t_boot_ms) const { return offset_ + (double)t_boot_ms * 1e-3; }
    // 反向：把本地秒时钟换成飞控的 time_boot_ms。
    // 协议 2.4.13 的 SET_POSITION_TARGET_LOCAL_NED.time_boot_ms 应该用飞控开机时钟，
    // 不能直接填本地 now*1000，否则飞控侧看起来是"未来/过去"的时间戳。
    uint32_t bootMs(double t_host_s) const
    {
      const double ms = (t_host_s - offset_) * 1e3;
      return (uint32_t)(ms > 0.0 ? ms : 0.0);
    }

  private:
    double offset_ = 0.0;
    bool valid_ = false;
  };

  // ------------------------------------------------------ 位姿组装（输入侧）
  // 每条GLOBAL_POSITION_INT给出一个真实位置+速度锚点。
  // 姿态到达即可按最近的非未来锚点外推位置，不等待后一帧定位。
  struct PoseAssemblerConfig
  {
    double max_position_extrapolation_s = 0.25;
  };

  class PoseAssembler
  {
  public:
    void setOrigin(const GeoOrigin &o) { origin_ = o; }
    void setConfig(const PoseAssemblerConfig &c) { cfg_ = c; }
    bool originValid() const { return origin_.valid; }
    // 最近一次位置流给出的 ENU 位置（未经姿态流外推）
    const Eigen::Vector3d &positionEnu() const { return pos_enu_; }
    const Eigen::Vector3d &velocityEnu() const { return vel_enu_; }
    bool hasPosition() const { return has_pos_; }

    // 位置流：得到 ENU 位置 + ENU 速度
    void onGlobalPositionInt(const GlobalPositionInt &g)
    {
      if (!origin_.valid)
        return;
      const double boot = (double)g.time_boot_ms * 1e-3;
      if (!position_history_.empty() && boot <= position_history_.back().t) return;
      const Eigen::Vector2d xy = geoToEnuXY(g.lat * 1e-7, g.lon * 1e-7, origin_);
      pos_enu_ = Eigen::Vector3d(xy.x(), xy.y(), g.relative_alt * 1e-3);
      vel_enu_ = nedToEnu(Eigen::Vector3d(g.vx, g.vy, g.vz) * 1e-2);
      has_pos_ = true;
      ego_sensor::PositionAnchor sample;
      sample.t=boot; sample.p=pos_enu_; sample.v=vel_enu_;
      sample.max_extrapolation_s=cfg_.max_position_extrapolation_s;
      sample.valid=true;
      position_history_.push_back(sample);
      while(position_history_.size()>500) position_history_.pop_front();
    }

    // 姿态流：按 t_boot_s 给出一个完整位姿（位置做外推）
    bool onAttitude(const Attitude &a, double t_boot_s, ego_sensor::TimedPose &out)
    {
      if (!has_pos_)
        return false;
      if (!std::isfinite(t_boot_s)) return false;
      // 历史只用于处理串口批次中较晚收到的旧姿态，绝不使用未来定位倒推。
      auto upper=std::upper_bound(position_history_.begin(),position_history_.end(),t_boot_s,
        [](double time,const ego_sensor::PositionAnchor &v){return time<v.t;});
      if (upper==position_history_.begin()) return false;
      const auto &anchor=*(upper-1);
      if (!anchor.positionAt(t_boot_s,out.p)) return false;
      out.position_anchor=anchor;
      out.q = attitudeNedToEnuBody(a.roll, a.pitch, a.yaw);
      // 位姿自己的时刻：逐点去畸变要靠它把每个点对到"它采样那一刻"的位姿上，
      // 缺了这一步整条位姿历史的时间戳都是 0。
      out.t = t_boot_s;
      return true;
    }

  private:
    GeoOrigin origin_;
    PoseAssemblerConfig cfg_;
    Eigen::Vector3d pos_enu_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d vel_enu_ = Eigen::Vector3d::Zero();
    bool has_pos_ = false;
    std::deque<ego_sensor::PositionAnchor> position_history_;
  };

  // ------------------------------------------------------ 设定值（输出侧）
  struct SetpointConfig
  {
    uint8_t target_system = 1;    // 飞机 ID（多机时各不相同）
    uint8_t target_component = 1;
    // MAVLink POSITION_TARGET_TYPEMASK：bit0-2=XYZ, 3-5=VXYZ, 6-8=AXYZ, 10=yaw, 11=yaw_rate
    // 默认"位置 + 速度前馈 + yaw"，忽略加速度与 yaw_rate
    uint16_t type_mask = (uint16_t)((1 << 6) | (1 << 7) | (1 << 8) | (1 << 11));
    uint8_t coordinate_frame = 1; // MAV_FRAME_LOCAL_NED
  };

  // QuadrotorCommand(ENU) -> SET_POSITION_TARGET_LOCAL_NED(NED)
  // 注意：QuadrotorCommand 的 kx/kv（位置/速度反馈增益）与 trajectory_id/flag 在
  // ID=84 里没有对应字段 —— 位置环增益由飞控内部负责，直接丢弃。
  inline void toSetpoint(const ego_planner::QuadrotorCommand &cmd, uint32_t time_boot_ms,
                         const SetpointConfig &cfg, SetPositionTargetLocalNed &out)
  {
    const Eigen::Vector3d p = enuToNed(cmd.position);
    const Eigen::Vector3d v = enuToNed(cmd.velocity);
    const Eigen::Vector3d a = enuToNed(cmd.acceleration);
    out.time_boot_ms = time_boot_ms;
    out.x = (float)p.x(); out.y = (float)p.y(); out.z = (float)p.z();
    out.vx = (float)v.x(); out.vy = (float)v.y(); out.vz = (float)v.z();
    out.afx = (float)a.x(); out.afy = (float)a.y(); out.afz = (float)a.z();
    out.yaw = (float)yawEnuToNed(cmd.yaw);
    out.yaw_rate = (float)yawRateEnuToNed(cmd.yaw_dot);
    out.type_mask = cfg.type_mask;
    out.target_system = cfg.target_system;
    out.target_component = cfg.target_component;
    out.coordinate_frame = cfg.coordinate_frame;
  }

} // namespace ego_mav

#endif
