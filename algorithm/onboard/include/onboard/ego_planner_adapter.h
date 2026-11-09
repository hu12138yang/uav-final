#ifndef _ONBOARD_EGO_PLANNER_ADAPTER_H_
#define _ONBOARD_EGO_PLANNER_ADAPTER_H_

// 把 ego_planner_core（EGO 局部规划 + 滚动占据栅格）接到 OffboardPilot 的 PlannerIO 上。
// 这是"我们的系统"与"飞控协议层"之间**唯一**的接线点：
//
//   飞控位姿     ──onPose──────► 位姿缓冲（供逐点去畸变）
//   雷达原始帧   ──onScanFrame─► 逐点去畸变 ─► 世界系点云 ─► GridMap（insert-only 滚动窗口）
//   当前航段 A/B ──setLeg──────► A→B 直线参考，内部切局部目标
//   100 Hz       ──step───────► 需要时重规划 ─► TrajectoryTracker ─► QuadrotorCommand
//
// 局部目标沿用仿真里已验证过的策略（不新增算法）：
//   1) 名义：沿 A→B 自当前投影点前视 `planning_horizon`；
//   2) 若落点**已知被占**：沿 A→B 继续前走，直到碰到空点；
//   3) 仍被占：整体向上抬，让 EGO 从障碍上方越过（大半径 / 贴线障碍）。
//   前视距离与让步上限都封在"当前航段"内，不越过本段终点，保证上层
//   OffboardPilot 的航点推进判定（wp_reach_tol）语义不变。

#include <Eigen/Eigen>

#include <memory>
#include <mutex>
#include <vector>

#include <ego_core/ego_local_planner.h>
#include <ego_core/trajectory_tracker.h>
#include <plan_env/grid_map.h>
#include <sensor/lidar_types.h>
#include <sensor/scan_assembler.h>

#include "onboard/offboard_pilot.h" // PlannerIO

namespace ego_onboard
{

  struct EgoPlannerAdapterConfig
  {
    /* ---- 滚动占据栅格 ---- */
    double resolution = 0.2;              // 体素边长（改参数不改算法：0.1 -> 0.2）
    // 高度带：基准**不是常数**，而是"开始导航那一帧"飞控报的实际高度
    //（接管瞬间的 cur_pos.z()，就是相对起飞点的离地高度）。
    // 向上 + altitude_band_half，向下 - max_descent_below_ref。
    // 滚动窗口只平移 x/y，z 范围必须一次定死，所以基准在第一次 step() 时才取定。
    double altitude_band_half = 10.0;     // 高度带向上半宽
    // 高度带下界低于巡航基准的量。注意：虚拟地板板层从 band_lo 往上铺
    // 1 个膨胀厚度，所以实际可飞下界 ≈ band_lo + obstacles_inflation。
    // 大机体贴近高塔/高墙时，LiDAR 看不到障碍下半部分；不允许大幅下潜，
    // 否则规划器会钻进"未知"的下部空间。生产 YAML 与膨胀联调以保持下降空间。
    double max_descent_below_ref = 2.0;
    double map_margin_z = 2.0;            // 高度带外再留的余量（板层厚度）
    // 最低离地高度（相对起飞点）。下界 = max(基准 - band, min_flight_height)，
    // 用飞控的 relative_alt 直接判 —— 基准低的时候高度带自动被顶起来，不会贴着地飞。
    double min_flight_height = 5.0;
    Eigen::Vector2d map_size_xy = Eigen::Vector2d(60.0, 60.0); // 滚动窗口 x/y 尺寸
    double occupancy_update_range_xy = 9.0; // 只有机体附近的体素会被刷新
    double occupancy_update_range_z = 6.0;
    double obstacles_inflation = 1.0;     // 必须 >= 机体包络半径
    int local_map_margin = 10;
    double recenter_margin = 8.0;         // 机体离窗口中心超过此值才滑动窗口
    // 连续丢多少帧判"感知失效"（10Hz 下默认 10 帧 = 1 秒）。
    // 丢帧意味着点云没进图，地图里全是"未知"，EGO 会规划直线 —— 必须让上层停下来。
    int sensing_drop_frames = 10;
    double sensing_timeout_s = 0.5;
    bool strict_pose_coverage = false;

    /* ---- 规划器 ---- */
    double max_vel = 5.0;                 // = 巡航速度，EGO 一直以它跑
    double max_acc = 6.0;
    double planning_horizon = 6.0;        // 局部目标前视距离
    double local_target_step = 1.0;       // 落点被占时沿航线前走的步长
    double local_target_max_ratio = 1.5;  // 前视上限 = ratio * planning_horizon
    double replan_period = 0.5;           // 周期性重规划
    double goal_tolerance = 0.5;          // 到达局部目标（本段终点）的判定
    double obstacle_clearance = 0.5;      // 优化器里的 dist0 / swarm_clearance
    double ctrl_pt_dist = 0.4;
    // A* 搜索池。注意搜索半径与分辨率无关：core 里以**硬编码**步长 0.1 m 调用
    // AstarSearch（bspline_optimizer.cpp），所以半程 = (pool_size/2) × 0.1 m，
    // 默认 100 即 ±5 m。5 m 够用 —— A* 只在碰撞段两端（间隔几米）找一条绕行引导，
    // 不负责全程寻路；局部目标的前视距离也不超过 9 m。
    Eigen::Vector3i astar_pool_size = Eigen::Vector3i(100, 100, 100);

    /* ---- 输出跟踪 ---- */
    double time_forward = 1.0;            // TrajectoryTracker 前视（yaw 平滑）

    /* ---- 外参：雷达 -> 机体 ---- */
    ego_sensor::Extrinsic extrinsic;
  };

  class EgoPlannerAdapter : public PlannerIO
  {
  public:
    EgoPlannerAdapter();
    ~EgoPlannerAdapter() override;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // 保存配置（调用一次）。**地图此时还没建** —— 高度带的基准要等第一次 step()
    // 拿到"开始导航那一帧"的实际高度才能定，在那之前 onScanFrame() 会直接返回。
    void init(const EgoPlannerAdapterConfig &cfg);

    // 新任务前清空地图与轨迹（下一次 step() 会用当时的高度重新建图）
    void reset();
    void prepareMap(double reference_height);
    bool frameCovered(const ego_sensor::LidarFrame &frame) const;

    /* ---------------- PlannerIO ---------------- */

    void setLeg(const Eigen::Vector3d &start, const Eigen::Vector3d &end, bool is_last) override;
    void onPose(const ego_sensor::TimedPose &pose) override;
    void onScanFrame(const ego_sensor::LidarFrame &frame) override;
    bool step(double now, const Eigen::Vector3d &cur_pos,
              ego_planner::QuadrotorCommand &cmd) override;

    // 丢弃当前轨迹：丢星悬停后恢复时用（旧轨迹早已过期，不能再拿来跟踪）
    void dropTrajectory() override;

    // 感知是否健康（连续丢帧 → 失效）。供状态机做安全悬停判断。
    bool sensingHealthy() const override;

    /* ---------------- 诊断 ---------------- */

    struct Stats
    {
      long frames_total = 0;   // 收到的原始帧
      long frames_dropped = 0; // 位姿覆盖不足而丢掉的帧
      long cloud_points = 0;   // 累计进图点数
      uint64_t step_calls = 0, blocked_start_holds = 0;
      bool start_blocked = false;
      int replan = 0;          // 重规划次数
      int plan_fail = 0;       // 规划失败次数
      int recenter = 0;        // 滑动窗口次数
      int consecutive_dropped = 0; // 当前连续丢帧数
      bool sensing_dead = false;   // 连续丢帧超阈值 → 感知失效
      double last_plan_ms = 0.0;
      double plan_ms_max = 0.0;
      // 地图插入耗时（实测口径：RK3588 上决定 update_range 能开多大）
      double inflate_ms_sum = 0.0;
      double inflate_ms_max = 0.0;
      int inflate_samples = 0;
    };
    const Stats &stats() const { return stats_; }

    GridMap::Ptr gridMap() const { return grid_map_; }
    ego_sensor::ScanAssembler &scanAssembler() { return scan_asm_; }
    bool haveTrajectory() const { return have_traj_; }
    bool braking() const { return braking_; }
    // 地图是否已经建好（= 第一次 step() 已经拿到高度基准）
    bool mapReady() const { return grid_map_ != nullptr; }
    // 高度基准（相对起飞点），未建图时为 0
    double altitudeRef() const { return z_ref_; }

  private:
    // 用给定的高度基准建图 + 建规划器（调用方必须已持有 mtx_）
    void buildMap(double z_ref);
    // 清空全部运行状态并释放地图（调用方必须已持有 mtx_）
    void clearState();

    // 沿参考航线的弧长：投影 / 取点 / 取方向
    double projectRoute(double s_lo, double s_hi, const Eigen::Vector3d &p, double &s_out) const;
    Eigen::Vector3d routeAt(double s) const;
    Eigen::Vector3d routeDirAt(double s) const;
    Eigen::Vector3d localTargetAt(double s_cur, double &s_target) const;
    bool trajCollidesAhead(double now);

    EgoPlannerAdapterConfig cfg_;
    mutable std::mutex mtx_;
    bool inited_ = false;
    double z_ref_ = 0.0; // 高度基准 = 开始导航那一帧的实际高度（相对起飞点）

    GridMap::Ptr grid_map_;
    ego_planner::EgoLocalPlanner planner_;
    ego_planner::TrajectoryTracker tracker_;
    ego_sensor::ScanAssembler scan_asm_;
    ego_sensor::PoseBuffer pose_buf_;

    // 当前航段 A→B（两个点）+ 累计弧长
    std::vector<Eigen::Vector3d> route_;
    std::vector<double> route_cum_;
    double route_total_ = 0.0;

    // 当前航段在 route_ 上的范围（两段点固定为 [0, total]）
    double leg_start_s_ = 0.0;
    double leg_end_s_ = 0.0;
    bool leg_last_ = false; // 本段是最后一段（到达 B 后任务完成）

    // 跟踪状态
    bool have_traj_ = false;
    bool braking_ = true;
    ego_planner::BsplineTrajectory traj_;
    double last_plan_ = -1e9;
    double last_safety_ = -1e9;
    bool collides_ahead_ = false;
    ego_planner::QuadrotorCommand last_cmd_;
    bool have_cmd_ = false;

    Eigen::Vector3d last_pose_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d last_recenter_ = Eigen::Vector3d::Zero();
    bool have_pose_ = false;

    Stats stats_;
    double newest_pose_time_ = -1e9;
    double last_scan_time_ = -1e9;
  };

} // namespace ego_onboard

#endif
