#ifndef _ONBOARD_OFFBOARD_PILOT_H_
#define _ONBOARD_OFFBOARD_PILOT_H_

// 板载 Offboard 飞行主控 —— 把我们的规划系统接到飞控协议上。
//
// 采用「航点间全程 Offboard」形态（不再反复切模式）：
//
//   S0 链路      : 发心跳 / 提频 / 等 HOME 与位置
//   S1 航线      : 参考航点顺序连接，未闭环时追加首点，所有返回仍走 EGO
//   S2 上传航线  : 44 -> 51/73 -> 47；只含速度(178)和普通航点(16)
//   S3 等地面站  : 不发送解锁/起飞；观察已解锁、巡航高度、定点模式
//   S4 接管      : 先预发送 setpoint，再请求 Offboard
//   S5 巡航/返回 : 按航点依次推进，最后普通航段返回 KML 起点
//   S6 收尾      : 起点上方悬停，降落由地面站负责；不发送 RTL/降落/上锁
//
// 规划层通过 PlannerIO 接口接入，便于离线用假规划器验证整条时序。

#include <cstdint>
#include <vector>

#include <Eigen/Eigen>

#include <ego_core/trajectory_tracker.h>
#include <sensor/lidar_types.h>

#include "mavlink/fcu_interface.h"
#include "mavlink/mavlink_codec.h"

#include "onboard/gps_dead_reckoning.h"
#include "onboard/time_alignment.h"

namespace ego_onboard
{
  using namespace ego_mav; // 本模块内部直接用协议层的名字

  // 航点（经纬高：高度是相对起飞点）
  struct GeoWaypoint
  {
    double lat_deg = 0, lon_deg = 0, alt_rel_m = 0;
  };

  // 规划层接口：由我们的 EGO 栈实现；测试里用假的实现即可验证时序
  struct PlannerIO
  {
    virtual ~PlannerIO() {}
    // 参考航线（局部 ENU，原点 = home）
    virtual void setReferenceRoute(const std::vector<Eigen::Vector3d> &) {}
    // 当前航段终点（局部 ENU）
    virtual void setLegGoal(const Eigen::Vector3d &) {}
    virtual void setLeg(const Eigen::Vector3d &, const Eigen::Vector3d &end, bool) { setLegGoal(end); }
    // 新位姿（来自飞控）
    virtual void onPose(const ego_sensor::TimedPose &pose) = 0;
    // 原始雷达坐标系点云；采样时间已映射到共同轴，由适配层逐点去畸变。
    virtual void onScanFrame(const ego_sensor::LidarFrame &frame) = 0;
    // 推进一拍规划，输出飞控指令；返回 false 表示暂时没有可用轨迹
    virtual bool step(double now, const Eigen::Vector3d &cur_pos,
                      ego_planner::QuadrotorCommand &cmd) = 0;
    // 丢弃当前轨迹（例如丢星悬停后恢复，旧轨迹已经过期）。
    // 默认什么都不做，假规划器不必实现。
    virtual void dropTrajectory() {}
    // 感知是否健康。连续丢帧 = 点云没进图，避障已经形同虚设，必须停下来。
    // 默认 true，假规划器不必实现。
    virtual bool sensingHealthy() const { return true; }
  };

  // 进入安全悬停的原因。
  // 注意：**规划失败不在这里** —— 规划层在失败时本来就会输出悬停指令并持续重试，
  // 切到 HOLD 反而会停掉重试，靠近障碍时会来回抖。
  enum class HoldReason
  {
    NONE = 0,
    GPS_LOST,     // 定位失效（fix_type 不够 / 收不到 GPS_RAW_INT）
    SENSOR_LOST,  // 感知失效（连续丢帧，点云进不了图）
    MISSION_DONE, // 最后一个航点完成，悬停等待人工接管（不会自动恢复）
  };
  const char *holdReasonName(HoldReason r);

  enum class Phase
  {
    IDLE = 0,
    LINK,
    MISSION_UPLOAD,
    WAIT_GROUND_START, // 只等地面站起飞到巡航高度并进入定点，不发送起飞/解锁
    PRE_OFFBOARD,   // 连续发 setpoint，准备切 Offboard
    OFFBOARD_CRUISE,// 航点间 Offboard 巡航（我们的系统接管）
    HOLD,           // 安全悬停：任务异常 / 任务完成都停在这里
    FAILED,         // 失败（err_ 里有原因）
  };
  const char *phaseName(Phase p);

  class OffboardPilot
  {
  public:
    struct Config
    {
      // 身份
      uint8_t sysid = 245;          // 板载计算机
      uint8_t compid = 1;
      uint8_t target_system = 1;    // 飞机 ID
      uint8_t target_component = 1;
      // 时序
      double heartbeat_hz = 1.0;
      double link_timeout = 3.0;     // 多久收不到飞控心跳算失联
      double pre_offboard_stream_s = 1.5;
      double command_ack_timeout = 2.0;
      int max_retry = 3;
      // 任务
      double cruise_alt_m = 50.0; // 仅用于地面站起飞后接管的高度检查
      double cruise_speed = 5.0;
      double handover_alt_tolerance_m = 1.0;
      double handover_position_tolerance_m = 5.0;
      double wp_reach_tol = 1.5;     // 航点到达判定
      // 起降由地面站负责；沿普通航点返回 KML 首点，在巡航高度悬停。
      // 规划
      bool use_planner = true;       // false = 只跑时序（离线验证用）

      // 提频（协议 2.5.1：req_stream_id 填 = 报文ID）
      // 默认 2Hz(GLOBAL_POSITION_INT) / 10Hz(ATTITUDE) 不足以支撑 100Hz 控制与去畸变
      uint16_t rate_global_position_hz = 50; // 33
      uint16_t rate_attitude_hz = 100;       // 30
      uint16_t rate_extension_hz = 10;       // 245 EXTENDED_SYS_STATE
      bool request_rates = true;

      // ---- 飞行任务异常 -> 安全悬停 ----
      // 所有"数据不可信 / 算不出来"的异常统一进 Phase::HOLD：
      // 冻结 setpoint 原地悬停、不做任何规划；位置/姿态估计仍继续并通过 UDP 上报。
      // 无法悬停的异常（掉出 Offboard、飞控失联、没有位置）直接 FAILED 报错。
      bool gps_failsafe = true;
      uint8_t gps_min_fix = 3;            // fix_type >= 3 才算有效定位（0/1 无定位，2 仅 2D）
      double gps_loss_confirm_s = 0.5;    // 连续无效多久判丢星（GPS_RAW_INT 是 2Hz）
      double gps_rx_timeout_s = 2.0;      // 收不到 GPS_RAW_INT 也按丢星处理

      bool sensing_failsafe = true;       // 连续丢帧（点云进不了图）-> 悬停

      // 异常全部清除后，稳定多久才恢复巡航
      double hold_recover_confirm_s = 2.0;

      // 输出形态：位置 + yaw 必发；速度前馈可选。
      // 关掉后是纯位置控制，由飞控自己闭环 —— 更简单、更不受我们速度估计误差影响，
      // 但跟踪会"肉"（位置环带宽低），有速度变化时会滞后，避障余量要相应加大。
      bool velocity_feedforward = false;
      bool observe_only = false;
      bool require_timesync = false; // main enables for ALL physical operation
      double pose_timeout_s = 0.25;
      double setpoint_hz = 20.0;
      
    };

    void setPlanner(PlannerIO *p) { planner_ = p; }
    void setConfig(const Config &c) { cfg_ = c; PoseAssemblerConfig pc; pc.strict_position_brackets=c.require_timesync; pose_asm_.setConfig(pc); }
    const Config &config() const { return cfg_; }

    // 航线以「第 1 个航点 = 起飞点」约定传入（与 KML 一致）
    void start(const std::vector<GeoWaypoint> &route);
    void startObserving() { cfg_.observe_only = true; phase_ = Phase::LINK; t_start_ = -1; }
    void setReceiveTime(double now) { t_now_ = now; }
    void setInputReady(bool ready) { input_ready_ = ready; }
    const TimeAlignment &timeAlignment() const { return alignment_; }
    bool poseFresh(double now) const { return have_pos_ && (!cfg_.require_timesync || (now >= latest_pose_.t && now-latest_pose_.t <= cfg_.pose_timeout_s)) && now - t_last_pos_ <= cfg_.pose_timeout_s && now - t_last_att_ <= cfg_.pose_timeout_s; }
    bool clockReady(double now) const { return !cfg_.require_timesync || alignment_.valid(now); }
    void stop(const std::string &why) { fail(why); }
    double fcuUnixTime() const { return fcu_unix_; }


    // 飞控下行字节（TCP/UDP 收上来的原始流）
    void onMavlinkBytes(const uint8_t *data, size_t len);

    // 周期调用：输出待发送的完整 MAVLink 帧
    void tick(double now, std::vector<std::vector<uint8_t>> &out);

    Phase phase() const { return phase_; }
    bool armed() const { return armed_; }
    uint8_t mainMode() const { return main_mode_; }
    uint8_t subMode() const { return sub_mode_; }
    const GeoOrigin &origin() const { return origin_; }
    int currentLeg() const { return leg_; }
    int routeSize() const { return (int)route_.size(); }
    const char *lastError() const { return err_.c_str(); }
    // 最近一次用于控制的实时位姿（局部 ENU，已按姿态流外推）。
    // 感知层用它确定传感器所在位置（合成点云 / 建图观测点都以此为准）。
    const ego_sensor::TimedPose &latestPose() const { return latest_pose_; }
    bool hasPosition() const { return have_pos_; }
    // GPS 健康状态（诊断用）
    bool gpsValid() const { return gps_valid_; }
    uint8_t gpsFixType() const { return gps_fix_type_; }
    // 飞行任务异常 / 安全悬停（诊断用）
    bool holding() const { return phase_ == Phase::HOLD; }
    HoldReason holdReason() const { return hold_reason_; }
    int holdCount() const { return hold_count_; }           // 累计进悬停次数
    int gpsDropCount() const { return gps_drop_count_; }    // 其中定位引起的
    int sensingDropCount() const { return sensing_drop_count_; } // 其中感知引起的
    const Eigen::Vector3d &holdPosition() const { return hold_pos_; }
    // 丢星后的推算经纬高（只用于上报/判点，不参与控制）；age_s = 距最后一次有效定位
    bool gpsEstimate(GeoFix &fix, double &age_s) const { return reckoner_.estimate(t_now_, fix, age_s); }
    // 静态诊断：各报文解析/CRC 统计
    uint64_t crcErrors() const { return parser_.crcErrors(); }
    const std::map<uint32_t, uint64_t> &crcErrorsByMsgid() const { return parser_.crcErrorsByMsgid(); }

  private:
    // 各报文处理
    void onHeartbeat(const Heartbeat &hb, double now);
    void onHomePosition(const HomePosition &hp);
    void onGlobalPositionInt(const GlobalPositionInt &g);
    void onGpsRawInt(const GpsRawInt &g);
    void onAttitude(const Attitude &a);
    void publishAttitudes();
    void onMissionRequestInt(const MissionRequestInt &r, std::vector<std::vector<uint8_t>> &out);
    void onMissionAck(const MissionAck &ack);
    void onCommandAck(const CommandAck &ack);

    // 发送辅助
    template <typename Msg>
    void send(const Msg &m, std::vector<std::vector<uint8_t>> &out)
    {
      uint8_t buf[255];
      m.encode(buf);
      auto f = pack(cfg_.sysid, cfg_.compid, seq_++, Msg::ID, buf, Msg::LEN, true);
      if (!f.empty())
        out.push_back(std::move(f));
    }
    void sendHeartbeat(std::vector<std::vector<uint8_t>> &out, double now);
    void fail(const std::string &why);

    Config cfg_;
    PlannerIO *planner_ = nullptr;
    Parser parser_;
    // 接收路径上攒下的待发帧（tick 时统一 flush）
    std::vector<std::vector<uint8_t>> outbox_;
    double t_now_ = 0.0;

    Phase phase_ = Phase::IDLE;
    GeoOrigin origin_;
    ClockSync clock_;
    TimeAlignment alignment_;
    double t_last_home_request_ = -1e9;
    double t_last_sync_request_ = -1e9, t_last_pos_ = -1e9, t_last_att_ = -1e9;
    double last_pos_boot_ = -1, last_att_boot_ = -1, fcu_unix_ = 0;
    bool input_ready_ = true;
    std::deque<Attitude> pending_attitudes_;

    PoseAssembler pose_asm_;

    std::vector<GeoWaypoint> route_;      // 经纬高（含起飞点）
    std::vector<bool> mission_sent_;
    std::vector<MissionItemInt> mission_;  // 下发飞控
    std::vector<Eigen::Vector3d> route_enu_;

    SetpointConfig sp_cfg_;
    size_t mission_cursor_ = 0;
    int leg_ = 0;              // 当前航段序号（route_[leg_] -> route_[leg_+1]）
    uint8_t seq_ = 0;

    double t_start_ = 0.0;
    double t_phase_ = 0.0;
    double t_last_hb_ = -1e9;
    double t_last_fc_hb_ = -1e9;
    double t_last_setpoint_ = -1e9;
    double t_cmd_sent_ = -1e9;
    uint16_t last_cmd_ = 0;
    int retry_ = 0;
    bool mode_switch_sent_ = false;
    bool waiting_offboard_ = false;
    bool rates_requested_ = false;
    double hold_yaw_ = 0.0;

    bool armed_ = false;
    uint8_t main_mode_ = 0, sub_mode_ = 0;
    Eigen::Vector3d pos_enu_ = Eigen::Vector3d::Zero();  // 位置流原值（2Hz，滞后）
    Eigen::Vector3d pos_ctrl_ = Eigen::Vector3d::Zero(); // 外推后（10Hz），供规划/判点用
    ego_sensor::TimedPose latest_pose_;                  // 同 pos_ctrl_，但带姿态
    bool have_pos_ = false;

    // GPS 健康监测
    uint8_t gps_fix_type_ = 0;
    uint8_t gps_sats_ = 0;
    bool gps_valid_ = false;       // 当前是否判定为"有效定位"
    double t_last_gps_ = -1e9;     // 最近一次收到 GPS_RAW_INT
    double t_gps_bad_ = -1e9;      // 开始连续无效的时刻
    double t_gps_good_ = -1e9;     // 开始连续有效的时刻

    // 安全悬停
    Eigen::Vector3d hold_pos_ = Eigen::Vector3d::Zero(); // 悬停点
    HoldReason hold_reason_ = HoldReason::NONE;
    int hold_count_ = 0;
    int gps_drop_count_ = 0;
    int sensing_drop_count_ = 0;
    double t_hold_clear_ = -1e9; // 所有异常都清除的起始时刻
    bool sensing_healthy_ = true;
    GpsDeadReckoner reckoner_;   // 丢星后的短时航位推算（只用于上报）

    std::string err_;
  };

} // namespace ego_onboard

#endif
