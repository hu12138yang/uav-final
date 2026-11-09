#include "onboard/ego_planner_adapter.h"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace ego_onboard
{

  namespace
  {
    double msSince(const std::chrono::steady_clock::time_point &t0)
    {
      return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    }
  } // namespace

  EgoPlannerAdapter::EgoPlannerAdapter() {}
  EgoPlannerAdapter::~EgoPlannerAdapter() {}

  void EgoPlannerAdapter::init(const EgoPlannerAdapterConfig &cfg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    cfg_ = cfg;
    inited_ = true;
    // 只存配置，**不建图**：高度带的基准是"开始导航那一帧"的实际高度，
    // 现在还不知道，等第一次 step() 拿到 cur_pos.z() 再建。
    clearState();
  }

  void EgoPlannerAdapter::reset()
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!inited_)
      return;
    // 重新分配占据缓冲 = 清空上一趟任务留下的障碍（insert-only 地图不会自己遗忘）
    clearState();
  }

  void EgoPlannerAdapter::clearState()
  {
    grid_map_.reset();
    z_ref_ = 0.0;

    route_.clear();
    route_cum_.clear();
    route_total_ = 0.0;
    leg_start_s_ = leg_end_s_ = 0.0;
    leg_last_ = false;

    have_traj_ = false;
    braking_ = true;
    have_cmd_ = false;
    collides_ahead_ = false;
    last_plan_ = -1e9;
    last_safety_ = -1e9;

    pose_buf_ = ego_sensor::PoseBuffer();
    last_pose_.setZero();
    last_recenter_.setZero();
    have_pose_ = false;

    stats_ = Stats();
    newest_pose_time_ = last_scan_time_ = -1e9;
    pose_buf_.setStrictCoverage(cfg_.strict_pose_coverage);
  }

  void EgoPlannerAdapter::prepareMap(double reference_height) {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!grid_map_) buildMap(reference_height);
  }
  bool EgoPlannerAdapter::frameCovered(const ego_sensor::LidarFrame &frame) const {
    std::lock_guard<std::mutex> lk(mtx_);
    if (frame.points.empty()) return false;
    ego_sensor::TimedPose p;
    return pose_buf_.poseAt(frame.t_base, p) &&
           pose_buf_.poseAt(frame.t_base + frame.points.back().t_rel, p);
  }

  void EgoPlannerAdapter::buildMap(double z_ref)
  {
    z_ref_ = z_ref;

    // 高度带下界：基准 - 允许下降量，但不得低于最低离地高度。
    // 基准就是接管那一帧的 relative_alt（相对起飞点）。
    const double band_lo = std::max(z_ref - cfg_.max_descent_below_ref, cfg_.min_flight_height);
    const double band_hi = z_ref + cfg_.altitude_band_half;

    // 地图 z 窗口 = 高度带再外扩 margin（给天花板/地板板层留厚度）。
    // 滚动窗口只平移 x/y（见 recenter），所以 z 的范围在这里一次定死。
    const double map_z_lo = band_lo - cfg_.map_margin_z;
    const double z_size = (band_hi + cfg_.map_margin_z) - map_z_lo;

    grid_map_.reset(new GridMap);
    grid_map_->initMap(cfg_.resolution,
                       Eigen::Vector3d(cfg_.map_size_xy.x(), cfg_.map_size_xy.y(), z_size),
                       Eigen::Vector3d(cfg_.occupancy_update_range_xy, cfg_.occupancy_update_range_xy,
                                       cfg_.occupancy_update_range_z),
                       cfg_.obstacles_inflation,
                       band_hi + cfg_.map_margin_z, // 天花板：交给 initMap 夹到地图顶
                       map_z_lo,                    // 地图底
                       cfg_.local_map_margin, "world");
    // 虚拟地板 = 高度带下界：规划器不会规划到它下面（与 virtual_ceil_height 对称）
    grid_map_->setVirtualGroundHeight(band_lo);

    ego_planner::EgoLocalPlannerConfig pcfg;
    pcfg.max_vel = cfg_.max_vel;
    pcfg.max_acc = cfg_.max_acc;
    pcfg.ctrl_pt_dist = cfg_.ctrl_pt_dist;
    pcfg.planning_horizon = cfg_.planning_horizon;
    // A* 搜索池（半程 = (pool_size/2) × 0.1 m；步长在 core 里是硬编码的，这里配不了）
    pcfg.astar_pool_size = cfg_.astar_pool_size;
    pcfg.opt.lambda_smooth = 1.0;
    pcfg.opt.lambda_collision = 0.5;
    pcfg.opt.lambda_feasibility = 0.1;
    pcfg.opt.lambda_fitness = 1.0;
    pcfg.opt.dist0 = cfg_.obstacle_clearance;
    pcfg.opt.swarm_clearance = cfg_.obstacle_clearance;
    pcfg.opt.max_vel = pcfg.max_vel;
    pcfg.opt.max_acc = pcfg.max_acc;

    planner_.init(pcfg);
    planner_.setEnvironment(grid_map_);

    tracker_ = ego_planner::TrajectoryTracker();
    tracker_.setTimeForward(cfg_.time_forward);
    scan_asm_.setExtrinsic(cfg_.extrinsic);

    // 轨迹状态清零（航线与位姿历史保留：航线在开图前就已经下发）
    have_traj_ = false;
    braking_ = true;
    have_cmd_ = false;
    collides_ahead_ = false;
    last_plan_ = -1e9;
    last_safety_ = -1e9;
  }

  // ------------------------------------------------------------------ 航段
  void EgoPlannerAdapter::setLeg(const Eigen::Vector3d &start, const Eigen::Vector3d &end, bool is_last)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    route_.clear();
    route_.push_back(start);
    route_.push_back(end);
    route_cum_.resize(2);
    route_cum_[0] = 0.0;
    route_cum_[1] = (end - start).norm();
    route_total_ = route_cum_[1];

    // 两点的航段：s ∈ [0, |A→B|]
    leg_start_s_ = 0.0;
    leg_end_s_ = route_total_;
    leg_last_ = is_last;

    // 新航段，旧轨迹作废；下一拍立即重规划。
    // 保留 last_cmd_，这样悬停/起步时的 yaw 不会跳变。
    have_traj_ = false;
    braking_ = true;
    collides_ahead_ = false;
    last_plan_ = -1e9;
    last_safety_ = -1e9;
  }

  // ------------------------------------------------------------------ 输入
  void EgoPlannerAdapter::onPose(const ego_sensor::TimedPose &pose)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    pose_buf_.add(pose);
    newest_pose_time_ = pose.t;
    last_pose_ = pose.p;
    have_pose_ = true;
  }

  void EgoPlannerAdapter::onScanFrame(const ego_sensor::LidarFrame &frame)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (!grid_map_ || !have_pose_)
      return;

    stats_.frames_total++;

    // 1) 逐点去畸变 -> 世界系点云。位姿覆盖不了整帧时必须丢帧：
    //    insert-only 地图会把用错位姿建出来的点永久留下。
    std::vector<Eigen::Vector3d> cloud;
    if (!scan_asm_.toWorld(frame, pose_buf_, cloud) || cloud.empty())
    {
      stats_.frames_dropped++;
      ++stats_.consecutive_dropped;
      if (stats_.consecutive_dropped >= cfg_.sensing_drop_frames)
        stats_.sensing_dead = true; // 连续丢帧：地图不再更新，避障已失效
      return;
    }
    // 一帧成功就算恢复（上层还会再做一次时间确认）
    stats_.consecutive_dropped = 0;
    stats_.sensing_dead = false;
    last_scan_time_ = frame.t_base + frame.points.back().t_rel;

    // 2) 取该帧基准时刻的位姿当作观测点（一帧 100 ms，用最新位姿会偏）
    Eigen::Vector3d view = last_pose_;
    ego_sensor::TimedPose tp;
    if (pose_buf_.poseAt(frame.t_base, tp))
      view = tp.p;

    // 3) 滚动窗口跟着机体平移
    if ((view.head<2>() - last_recenter_.head<2>()).norm() > cfg_.recenter_margin)
    {
      grid_map_->recenter(view);
      last_recenter_ = view;
      stats_.recenter++;
    }

    // 4) 插入（insert-only：只把 0 写成 1，不做逐帧清空）
    grid_map_->sensedPoints(cloud, view);
    stats_.cloud_points += (long)cloud.size();
    // 实测插入耗时：决定 update_range / inflation 能开多大
    stats_.inflate_ms_sum += grid_map_->prof_inflate_ms;
    stats_.inflate_ms_max = std::max(stats_.inflate_ms_max, grid_map_->prof_inflate_ms);
    ++stats_.inflate_samples;
  }

  // ------------------------------------------------------------- 轨迹管理
  bool EgoPlannerAdapter::sensingHealthy() const
  {
    std::lock_guard<std::mutex> lk(mtx_);
    return stats_.inflate_samples > 0 && !stats_.sensing_dead &&
           newest_pose_time_ >= last_scan_time_ - 0.02 &&
           newest_pose_time_ - last_scan_time_ <= cfg_.sensing_timeout_s;
  }

  void EgoPlannerAdapter::dropTrajectory()
  {
    std::lock_guard<std::mutex> lk(mtx_);
    have_traj_ = false;
    braking_ = true;
    collides_ahead_ = false;
    last_plan_ = -1e9;
    last_safety_ = -1e9;
  }

  // ------------------------------------------------------------- 航线几何
  double EgoPlannerAdapter::projectRoute(double s_lo, double s_hi, const Eigen::Vector3d &p, double &s_out) const
  {
    if (route_.size() < 2)
    {
      s_out = 0.0;
      return route_.empty() ? 1e18 : (p - route_[0]).norm();
    }

    double best = 1e18;
    s_out = s_lo;
    for (size_t i = 1; i < route_.size(); ++i)
    {
      const double sa = route_cum_[i - 1], sb = route_cum_[i];
      if (sb < s_lo || sa > s_hi)
        continue;
      const Eigen::Vector3d a = route_[i - 1], ab = route_[i] - route_[i - 1];
      const double len2 = ab.squaredNorm();
      double u = len2 > 1e-12 ? (p - a).dot(ab) / len2 : 0.0;
      u = std::max(0.0, std::min(1.0, u));
      const double s = std::max(s_lo, std::min(s_hi, sa + u * std::sqrt(len2)));
      const double d = (p - routeAt(s)).squaredNorm();
      if (d < best)
      {
        best = d;
        s_out = s;
      }
    }
    if (best > 1e17)
    {
      s_out = 0.5 * (s_lo + s_hi);
      return (p - routeAt(s_out)).norm();
    }
    return std::sqrt(best);
  }

  Eigen::Vector3d EgoPlannerAdapter::routeAt(double s) const
  {
    if (route_.empty())
      return Eigen::Vector3d::Zero();
    if (route_.size() < 2)
      return route_.front();
    s = std::max(0.0, std::min(route_total_, s));
    size_t i = 1;
    while (i + 1 < route_cum_.size() && route_cum_[i] < s)
      ++i;
    const double seg = route_cum_[i] - route_cum_[i - 1];
    const double u = seg > 1e-9 ? (s - route_cum_[i - 1]) / seg : 0.0;
    return route_[i - 1] + (route_[i] - route_[i - 1]) * u;
  }

  Eigen::Vector3d EgoPlannerAdapter::routeDirAt(double s) const
  {
    if (route_.size() < 2)
      return Eigen::Vector3d(1.0, 0.0, 0.0);
    s = std::max(0.0, std::min(route_total_, s));
    size_t i = 1;
    while (i + 1 < route_cum_.size() && route_cum_[i] < s)
      ++i;
    const Eigen::Vector3d d = route_[i] - route_[i - 1];
    const double n = d.norm();
    return n > 1e-9 ? d / n : Eigen::Vector3d(1.0, 0.0, 0.0);
  }

  Eigen::Vector3d EgoPlannerAdapter::localTargetAt(double s_cur, double &s_target) const
  {
    // 前视上限：既不能超过"让步倍数"，也不能超过地图真正有数据的范围
    const double lt_max =
        std::min(cfg_.local_target_max_ratio * cfg_.planning_horizon, cfg_.occupancy_update_range_xy);

    double la = cfg_.planning_horizon;
    double s_t = std::min(s_cur + la, leg_end_s_);
    Eigen::Vector3d p = routeAt(s_t);

    // 落点是"已知被占"（==1）就沿航线继续前走；-1（窗口外=未知）不当障碍
    while (s_cur + la < leg_end_s_ && la < lt_max && grid_map_->getInflateOccupancy(p) == 1)
    {
      la = std::min(la + cfg_.local_target_step, lt_max);
      s_t = std::min(s_cur + la, leg_end_s_);
      p = routeAt(s_t);
    }

    // 还是被占（障碍比让步范围还宽 / 正压在航线上）：整体抬升，从上方越过。
    // 抬升上限跟着高度带的基准走，再留一个膨胀量，避免目标点贴到天花板板层上。
    if (grid_map_->getInflateOccupancy(p) == 1)
    {
      const double z_top = z_ref_ + cfg_.altitude_band_half - cfg_.obstacles_inflation;
      for (double z = p.z() + cfg_.local_target_step; z <= z_top; z += cfg_.local_target_step)
      {
        const Eigen::Vector3d q(p.x(), p.y(), z);
        if (grid_map_->getInflateOccupancy(q) != 1)
        {
          p = q;
          break;
        }
      }
    }

    s_target = s_t;
    return p;
  }

  bool EgoPlannerAdapter::trajCollidesAhead(double now)
  {
    if (!have_traj_)
      return false;
    const double t_cur = std::max(0.0, now - traj_.start_time);
    const double t_23 = traj_.duration * 2.0 / 3.0;
    double t_end = (t_cur < t_23) ? t_23 : traj_.duration;
    for (double tt = t_cur; tt <= t_end + 1e-6; tt += 0.01)
    {
      const Eigen::Vector3d q = traj_.bspline.evaluateDeBoorT(std::min(tt, traj_.duration));
      if (grid_map_->getInflateOccupancy(q) != 0)
        return true;
    }
    return false;
  }

  // ---------------------------------------------------------------- 规划
  bool EgoPlannerAdapter::step(double now, const Eigen::Vector3d &cur_pos,
                               ego_planner::QuadrotorCommand &cmd)
  {
    std::lock_guard<std::mutex> lk(mtx_);

    auto hold = [&]() {
      cmd.position = cur_pos;
      cmd.velocity.setZero();
      cmd.acceleration.setZero();
      cmd.yaw = have_cmd_ ? last_cmd_.yaw : 0.0;
      cmd.yaw_dot = 0.0;
      cmd.stamp = now;
      cmd.trajectory_flag = ego_planner::TRAJECTORY_STATUS_EMPTY;
      last_cmd_ = cmd;
      have_cmd_ = true;
    };

    // 第一次被调用 = OffboardPilot 刚接管，也就是"开始导航那一帧"。
    // 用这一帧的实际高度当高度带基准来建图（不是配置里的常数）。
    if (!grid_map_)
    {
      const double z_ref = (cur_pos.z() > 0.5) ? cur_pos.z() : (route_.empty() ? 0.0 : route_.back().z());
      buildMap(z_ref);
    }

    ++stats_.step_calls;
    stats_.start_blocked = grid_map_->getInflateOccupancy(cur_pos) != 0;
    if (stats_.start_blocked && !have_traj_)
    {
      // Invalid initial start is an input/map condition: hold rather than
      // spending time optimizing a trajectory from inside an inflated obstacle.
      ++stats_.blocked_start_holds;
      have_traj_ = false;
      braking_ = true;
      hold();
      return true;
    }

    if (route_.size() < 2)
    {
      braking_ = true;
      hold();
      return true;
    }

    double s_cur = 0.0;
    projectRoute(leg_start_s_, leg_end_s_, cur_pos, s_cur);

    double s_tgt = s_cur;
    const Eigen::Vector3d local_target = localTargetAt(s_cur, s_tgt);

    // 局部目标已经到达（最后一段时就是任务终点）：停在原地，由上层 OffboardPilot
    // 判点后推进航段 / 交还飞控。不再重规划，否则会不停报 "Close to goal"。
    if ((cur_pos - local_target).norm() < cfg_.goal_tolerance)
    {
      have_traj_ = false;
      braking_ = true;
      hold();
      return true;
    }

    // 轨迹安全扫描（20 Hz，同原 FSM 的 safety 定时器）
    if (have_traj_ && now - last_safety_ >= 0.05)
    {
      collides_ahead_ = trajCollidesAhead(now);
      last_safety_ = now;
    }

    bool need_plan;
    if (!have_traj_)
      need_plan = (now - last_plan_ >= 0.1); // 还没轨迹时限速重试
    else
      need_plan = collides_ahead_ || (now - last_plan_ >= cfg_.replan_period) ||
                  (traj_.start_time + traj_.duration - now <= 0.2);

    if (need_plan)
    {
      // 局部目标速度：原 FSM 传的是参考轨迹的速度（非零），让机体"穿过"前视点
      // 而不是停在上面；只有在本段就是最后一段且已进入刹车距离时才置零。
      Eigen::Vector3d local_target_vel;
      const double brake_d = cfg_.max_vel * cfg_.max_vel / (2.0 * cfg_.max_acc);
      if (leg_last_ && (route_.back() - local_target).norm() < brake_d)
        local_target_vel.setZero();
      else
        local_target_vel = routeDirAt(s_tgt) * cfg_.max_vel;

      // 起点状态：用上一次输出的速度/加速度（EGO 自己的输出 ≈ 实际状态，
      // 比用位置差分稳），第一次规划则为零。
      const Eigen::Vector3d v0 = have_cmd_ ? last_cmd_.velocity : Eigen::Vector3d::Zero();
      const Eigen::Vector3d a0 = have_cmd_ ? last_cmd_.acceleration : Eigen::Vector3d::Zero();

      ego_planner::BsplineTrajectory new_traj;
      // 首帧 = FSM GEN_NEW_TRAJ，之后 = FSM REPLAN_TRAJ
      const auto mode = have_traj_ ? ego_planner::PlanMode::REPLAN_TRAJ : ego_planner::PlanMode::NEW_TRAJ;
      const auto t0 = std::chrono::steady_clock::now();
      const bool ok = planner_.plan(cur_pos, v0, a0, local_target, local_target_vel, now, new_traj, mode);
      stats_.last_plan_ms = msSince(t0);
      stats_.plan_ms_max = std::max(stats_.plan_ms_max, stats_.last_plan_ms);

      if (ok)
      {
        traj_ = new_traj;
        tracker_.setTrajectory(traj_);
        have_traj_ = true;
        braking_ = false;
      }
      else
      {
        stats_.plan_fail++;
        // 规划不出来就原地悬停，绝不硬闯
        if (!have_traj_ || collides_ahead_ || (traj_.start_time + traj_.duration - now <= 0.2))
          braking_ = true;
      }
      last_plan_ = now;
      stats_.replan++;
    }

    if (!have_traj_)
      braking_ = true;

    if (braking_)
    {
      hold();
      return true;
    }

    if (!tracker_.getCommand(now, cmd))
    {
      braking_ = true;
      hold();
      return true;
    }

    last_cmd_ = cmd;
    have_cmd_ = true;
    return true;
  }

} // namespace ego_onboard
