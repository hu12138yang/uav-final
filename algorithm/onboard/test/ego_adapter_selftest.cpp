// EgoPlannerAdapter 自检：不需要 ROS、不需要飞控、不需要真雷达。
//
// 用"合成激光帧 + 合成位姿"喂适配器，看它是不是真的：
//   1) 把点云建成占据栅格（insert-only）；
//   2) 生成并跟踪一条避障轨迹；
//   3) 命令始终落在自由空间（不撞）；
//   4) 绕开障碍后回到参考航线并到达本段终点；
//   5) 位姿覆盖不足时整帧丢弃（绝不拿错位姿污染 insert-only 地图）。

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include <Eigen/Eigen>

#include "onboard/ego_planner_adapter.h"
#include "onboard/config_file.h"

using namespace ego_onboard;

static int g_fail = 0;
static void check(bool ok, const std::string &what)
{
  std::printf("  [%s] %s\n", ok ? " OK " : "FAIL", what.c_str());
  if (!ok)
    g_fail++;
}

int main(int argc, char **argv)
{
  if (argc != 2) return 2;
  EgoPlannerAdapterConfig cfg;
  cfg.resolution = 0.2;
  // 高度带基准不在这里配：它由第一次 step() 那一帧的实际高度决定
  cfg.altitude_band_half = 10.0;
  // 单测保留完整的上下对称带，专门验证适配层逻辑；
  // 实机 YAML 与膨胀量联调，保留较小的实际下降空间。
  cfg.max_descent_below_ref = 10.0;
  cfg.map_margin_z = 2.0;
  cfg.min_flight_height = 5.0;
  cfg.map_size_xy = Eigen::Vector2d(60.0, 60.0);
  cfg.obstacles_inflation = 1.0;
  cfg.max_vel = 5.0;
  cfg.planning_horizon = 6.0;
  cfg.local_target_step = 1.0;
  cfg.replan_period = 0.5;

  EgoPlannerAdapter adapter;
  adapter.init(cfg);

  // 当前航段：沿 +x 直线，高度 30 m（A=(0,0,30)，B=(30,0,30)）
  const Eigen::Vector3d start(0.0, 0.0, 30.0);
  const Eigen::Vector3d goal(30.0, 0.0, 30.0);
  adapter.setLeg(start, goal, true);

  // 障碍：正压在航线上的一个方块（x 12~18, y -5~5, z 24~32）
  std::vector<Eigen::Vector3d> block;
  for (double x = 12.0; x <= 18.001; x += 1.0)
    for (double y = -5.0; y <= 5.001; y += 1.0)
      for (double z = 24.0; z <= 32.001; z += 1.0)
        block.push_back(Eigen::Vector3d(x, y, z));

  const double dt = 0.01;        // 100 Hz 控制
  const double scan_dt = 0.1;    // 10 Hz 雷达
  const double T = 14.0;

  Eigen::Vector3d p = start;
  double next_scan = 0.0;
  double max_dev_y = 0.0, max_dev_z = 0.0, max_off_route = 0.0;
  bool ever_non_braking = false;
  bool cmd_in_obstacle = false;
  bool ever_occupied_seen = false;
  int steps = 0;

  for (double t = 0.0; t < T; t += dt)
  {
    // 1) 位姿（模拟飞控 100 Hz 上报；雷达与机体同原点同朝向）
    ego_sensor::TimedPose pose;
    pose.t = t;
    pose.p = p;
    pose.q = Eigen::Quaterniond::Identity();
    adapter.onPose(pose);

    // 2) 10 Hz 合成点云帧（观测点 = 机体位置，坐标系 = 机体，外参为单位阵）
    if (t >= next_scan)
    {
      next_scan += scan_dt;
      ego_sensor::LidarFrame frame;
      frame.t_base = t;
      frame.points.reserve(block.size());
      for (const auto &w : block)
      {
        ego_sensor::LidarPoint lp;
        lp.p = w - p; // R = I -> 传感器系点 = 世界点 - 位置
        lp.t_rel = 0.0;
        frame.points.push_back(lp);
      }
      adapter.onScanFrame(frame);
    }

    // 3) 规划一拍
    ego_planner::QuadrotorCommand cmd;
    const bool ok = adapter.step(t, p, cmd);
    if (!ok)
    {
      check(false, "step() 应当总是返回 true（失败时输出悬停指令）");
      break;
    }
    steps++;

    if (!adapter.braking())
      ever_non_braking = true;

    // 4) 命令不得落在占据体内
    if (adapter.gridMap()->getInflateOccupancy(cmd.position) == 1)
      cmd_in_obstacle = true;

    // 5) 一格近似的位置环（时间常数 ~0.2 s）
    p += (cmd.position - p) * std::min(1.0, 5.0 * dt);

    // 统计：偏离参考航线的程度
    const Eigen::Vector3d on_route(p.x(), 0.0, 30.0);
    max_off_route = std::max(max_off_route, (p - on_route).norm());
    max_dev_y = std::max(max_dev_y, std::fabs(p.y()));
    max_dev_z = std::max(max_dev_z, std::fabs(p.z() - 30.0));

    if (adapter.gridMap()->getInflateOccupancy(Eigen::Vector3d(15.0, 0.0, 30.0)) == 1)
      ever_occupied_seen = true;
  }

  const auto &st = adapter.stats();
  std::printf("  [info] steps=%d 帧 %ld(丢 %ld) 入图点 %ld 重规划 %d 失败 %d 滑窗 %d 末次规划 %.1f ms\n",
              steps, st.frames_total, st.frames_dropped, st.cloud_points,
              st.replan, st.plan_fail, st.recenter, st.last_plan_ms);
  std::printf("  [info] 最终位置 (%.2f, %.2f, %.2f)  最大偏航 %.2f m  最大 Δz %.2f m\n",
              p.x(), p.y(), p.z(), max_dev_y, max_dev_z);

  // ---- 断言 ----
  check(st.frames_total > 50, "收到了激光帧");
  check(ever_occupied_seen, "点云已建成占据栅格（障碍处 occupancy == 1）");
  check(st.frames_dropped == 0, "位姿覆盖充分，没有丢帧");
  check(ever_non_braking, "规划成功并生成了轨迹（非悬停）");
  check(st.plan_fail == 0, "没有规划失败");
  check(!cmd_in_obstacle, "所有位置指令都落在自由空间（不撞障碍）");
  check(max_dev_y > 1.0 || max_dev_z > 1.0, "确实产生了避障机动（发生了侧绕 / 上跨）");
  check((p - goal).norm() < 2.0, "绕开障碍后回到航线并到达本段终点");

  // ---- 位姿覆盖不足时必须丢帧 ----
  {
    EgoPlannerAdapter a2;
    a2.init(cfg);
    a2.setLeg(start, goal, true);
    ego_sensor::TimedPose pose;
    pose.t = 0.0;
    pose.p = start;
    pose.q = Eigen::Quaterniond::Identity();
    a2.onPose(pose);

    // 先跑一拍把图建起来（图要等"开始导航那一帧"才建，之前 onScanFrame 会被忽略）
    ego_planner::QuadrotorCommand c2;
    a2.step(0.0, start, c2);
    check(a2.mapReady(), "建图完成（第一次 step 之后）");

    ego_sensor::LidarFrame frame;
    frame.t_base = 100.0; // 位姿历史覆盖不到
    frame.points.push_back(ego_sensor::LidarPoint());
    a2.onScanFrame(frame);
    check(a2.stats().frames_dropped == 1, "位姿覆盖不足的帧被整帧丢弃");

    // 还没有航线/轨迹时也要给出安全的悬停指令
    EgoPlannerAdapter a3;
    a3.init(cfg);
    ego_planner::QuadrotorCommand cmd;
    check(a3.step(0.0, start, cmd) && cmd.velocity.norm() == 0.0, "无航线时输出悬停指令");
  }

  // ---- 高度带基准 = 开始导航那一帧的实际高度（不是写死的常数）----
  {
    EgoPlannerAdapter a4;
    a4.init(cfg);
    check(!a4.mapReady(), "init() 之后还没建图（要等开始导航那一帧）");

    const Eigen::Vector3d s4(0.0, 0.0, 12.0); // 这一趟的低空基准：12 m
    a4.setLeg(s4, Eigen::Vector3d(20.0, 0.0, 12.0), true);

    ego_planner::QuadrotorCommand cmd;
    a4.step(0.0, s4, cmd);
    check(a4.mapReady(), "第一次 step() 才建图");
    check(std::fabs(a4.altitudeRef() - 12.0) < 1e-9,
          "高度基准 = 开始导航那一帧的实际高度（12 m，不是配置里的 30 m）");
    // 下界 = max(12 - 10, min_flight_height=5) = 5，地图底 = 5 - margin(2) = 3，
    // 所以 1 m 处已经在地图外；若没做最低离地高度约束，地图底会是 0，那里就在图内。
    check(a4.gridMap()->getInflateOccupancy(Eigen::Vector3d(0.0, 0.0, 1.0)) == -1,
          "高度带下界被最低离地高度顶起来（基准 12 m 时不贴着地面开图）");
  }

  // Validate the production vehicle envelope and actual inflated map, including
  // the virtual floor/ceiling interactions after increasing rotor-tip clearance.
  {
    AppConfig app; std::string error;
    const bool loaded = loadConfigFile(argv[1], app, error);
    check(loaded, "生产安全间距参数可以加载: " + error);
    if (loaded) {
      const double inflated = std::ceil(app.adapter.obstacles_inflation / app.adapter.resolution) * app.adapter.resolution;
      check(inflated - app.vehicle_radius >= 1.5, "生产栅格膨胀覆盖机体及至少 1.5 m 桨尖余量");
      EgoPlannerAdapter production;
      auto live = app.adapter; live.strict_pose_coverage = true;
      production.init(live); production.prepareMap(50);
      ego_sensor::TimedPose pose; pose.t = 1; pose.p = Eigen::Vector3d(0,0,50);
      production.onPose(pose);
      ego_sensor::LidarFrame frame; frame.t_base = 1;
      ego_sensor::LidarPoint point; point.p = Eigen::Vector3d(8,0,0); frame.points.push_back(point);
      production.onScanFrame(frame);
      check(production.stats().inflate_samples == 1, "生产参数下点云成功入图");
      auto map = production.gridMap();
      check(map->getInflateOccupancy(pose.p) == 0, "增大膨胀后巡航高度不被虚拟地板封住");
      check(map->getInflateOccupancy(Eigen::Vector3d(0,0,49.8)) == 0 &&
            map->getInflateOccupancy(Eigen::Vector3d(0,0,49.5)) == 1,
            "虚拟地板保持原有下降边界");
      check(map->getInflateOccupancy(Eigen::Vector3d(0,0,60)) == 0 &&
            map->getInflateOccupancy(Eigen::Vector3d(0,0,60.3)) == 1,
            "虚拟顶板保持原有抬升边界");
      const double unsafe = app.vehicle_radius + app.min_obstacle_gap - 0.001;
      bool blocked = true;
      for (int axis = 0; axis < 3; ++axis)
        for (double direction : {-1.0, 1.0}) {
          Eigen::Vector3d center(8,0,50); center[axis] += direction * unsafe;
          blocked = blocked && map->getInflateOccupancy(center) == 1;
        }
      check(blocked, "六轴上桨尖不足 1.5 m 的中心位置均不可通行");
      check(map->getInflateOccupancy(Eigen::Vector3d(8-inflated-.25,0,50)) == 0,
            "膨胀边界外的已知空闲位置仍可通行");
    }
  }

  std::printf("\n%s (fail=%d)\n", g_fail == 0 ? "全部通过" : "存在失败", g_fail);
  return g_fail == 0 ? 0 : 1;
}
