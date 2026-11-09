// Offboard 全流程离线验证：用「假飞控 + 假规划器」把
// 「上传普通航点 → 等地面站起飞 → Offboard 航点与返回 → 起点悬停」
// 整条时序跑一遍，逐步校验板载发出的每一条报文。
// 假飞控在 test/fake_fcu.h 里（仅供离线自检使用）。
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include <onboard/offboard_pilot.h>
#include <onboard/route_utils.h>

#include "fake_fcu.h"

using namespace ego_mav;
using namespace ego_onboard;
using namespace ego_test;

namespace
{
  int g_fail = 0;
  void check(bool ok, const char *what)
  {
    std::printf("  [%s] %s\n", ok ? " OK " : "FAIL", what);
    if (!ok) g_fail++;
  }

  // 假规划器：直接朝航段终点飞（只为把时序跑通，不做避障）
  struct FakePlanner : PlannerIO
  {
    double speed = 5.0;
    std::vector<Eigen::Vector3d> route;
    Eigen::Vector3d goal = Eigen::Vector3d::Zero();
    bool have_goal = false;
    int poses = 0;

    void setReferenceRoute(const std::vector<Eigen::Vector3d> &r) override { route = r; }
    void setLegGoal(const Eigen::Vector3d &g) override { goal = g; have_goal = true; }
    void onPose(const ego_sensor::TimedPose &) override { ++poses; }
    void onScanFrame(const ego_sensor::LidarFrame &) override {}
    bool step(double, const Eigen::Vector3d &cur, ego_planner::QuadrotorCommand &cmd) override
    {
      if (!have_goal) return false;
      const Eigen::Vector3d d = goal - cur;
      const double dist = d.norm();
      Eigen::Vector3d v = Eigen::Vector3d::Zero();
      if (dist > 1e-6)
      {
        const double s = std::min(speed, std::sqrt(2.0 * 4.0 * dist));
        v = d / dist * s;
      }
      cmd.position = cur + v * 0.1;
      cmd.velocity = v;
      cmd.acceleration.setZero();
      cmd.yaw = 0.0;
      cmd.yaw_dot = 0.0;
      return true;
    }
  };
} // namespace

int main()
{
  // 航线：起飞点 + 3 个航点（与 KML 约定一致）；每段约 55 m，便于在 5 m/s 下快速跑完
  std::vector<GeoWaypoint> route;
  route.push_back({21.0000, 113.0000, 30.0});
  route.push_back({21.0005, 113.0000, 30.0});
  route.push_back({21.0010, 113.0000, 30.0});
  route.push_back({21.0015, 113.0000, 30.0});

  bool return_added = false;
  std::string route_error;
  check(ensureReturnToStart(route, 2000, return_added, route_error) && return_added, "开口航线自动追加返回首点");

  FakeFcu fcu;
  fcu.route = route;
  fcu.cruise = 5.0;

  FakePlanner planner;
  planner.speed = 5.0;

  OffboardPilot pilot;
  OffboardPilot::Config cfg;
  cfg.cruise_alt_m = 30.0;
  cfg.cruise_speed = 5.0;
  cfg.pre_offboard_stream_s = 1.0;
  cfg.wp_reach_tol = 1.5;
  cfg.heartbeat_hz = 2.0;
  pilot.setConfig(cfg);
  pilot.setPlanner(&planner);
  pilot.start(route);

  std::printf("== Offboard 全流程（假飞控 + 假规划器）==\n");
  const double dt = 0.1;
  Phase last = pilot.phase();
  std::vector<std::pair<Phase, double>> transitions;
  transitions.push_back({last, 0.0});

  int offboard_sp_frames = 0;
  bool saw_pre_offboard_first_sp = false;
  Eigen::Vector3d first_sp_enu = Eigen::Vector3d::Zero();
  int max_leg = 0;
  int done_ticks = 0;
  bool ground_started = false;
  bool observed_ground_wait = false;
  bool no_early_setpoints = true;
  std::vector<uint16_t> set_current_seqs;

  for (double now = 0.0; now < 360.0; now += dt)
  {
    if (pilot.phase() == Phase::FAILED)
      break;
    // 任务完成后让它再悬停几拍（验证 HOLD 会持续发 setpoint）
    if (pilot.holdReason() == HoldReason::MISSION_DONE && ++done_ticks > 5)
      break;

    std::vector<std::vector<uint8_t>> frames;
    if (pilot.phase() == Phase::WAIT_GROUND_START)
    {
      observed_ground_wait = true;
      if (!ground_started && now >= 10.0)
      {
        check(!fcu.armed && fcu.received_commands.empty() && fcu.sp_count == 0,
              "地面站操作前保持等待，无解锁/起飞/模式指令或 setpoint");
        fcu.simulateGroundStationTakeoff();
        ground_started = true;
      }
    }
    pilot.tick(now, frames);
    if (!ground_started) no_early_setpoints &= fcu.sp_count == 0;
    if (pilot.phase() == Phase::PRE_OFFBOARD || pilot.phase() == Phase::OFFBOARD_CRUISE)
    {
      // 统计 setpoint
      for (const auto &f : frames)
      {
        if (f.size() > 10 && f[0] == 0xFD)
        {
          const uint32_t id = f[7] | (f[8] << 8) | (f[9] << 16);
          if (id == SetPositionTargetLocalNed::ID)
          {
            ++offboard_sp_frames;
            if (!saw_pre_offboard_first_sp)
            {
              saw_pre_offboard_first_sp = true;
              const uint8_t *p = f.data() + 10;
              first_sp_enu = nedToEnu(Eigen::Vector3d(rdF32(p, 4), rdF32(p, 8), rdF32(p, 12)));
            }
          }
        }
      }
    }
    // 板载 -> 飞控
    std::vector<std::vector<uint8_t>> fcu_out;
    for (const auto &f : frames)
      fcu.onBytes(f.data(), f.size(), fcu_out);

    // 飞控推进 + 下行
    fcu.tick(dt, now, fcu_out);

    // 飞控 -> 板载
    for (const auto &f : fcu_out)
      pilot.onMavlinkBytes(f.data(), f.size());

    if (fcu.set_current_seen != 0xFFFF)
    {
      set_current_seqs.push_back(fcu.set_current_seen);
      fcu.set_current_seen = 0xFFFF;
    }
    max_leg = std::max(max_leg, pilot.currentLeg());

    if (pilot.phase() != last)
    {
      transitions.push_back({pilot.phase(), now});
      last = pilot.phase();
    }
  }

  std::printf("\n-- 状态迁移 --\n");
  for (auto &t : transitions)
    std::printf("   t=%6.1fs  %s\n", t.second, phaseName(t.first));

  std::printf("\n-- 飞控发出的报文（节选，fcu.log 只记飞控发出来的）--\n");
  for (size_t i = 0; i < fcu.log.size() && i < 14; ++i)
    std::printf("   %s\n", fcu.log[i].c_str());
  std::printf("   ...（共 %zu 条）\n", fcu.log.size());

  std::printf("\n-- 断言 --\n");
  if (pilot.phase() == Phase::FAILED)
    std::printf("  [FAIL] 流程失败: %s\n", pilot.lastError());
  g_fail += (pilot.phase() == Phase::FAILED) ? 1 : 0;

  // 状态迁移齐全
  bool has_link = false, has_upload = false, has_ground_wait = false, has_preoff = false;
  bool has_cruise = false, has_hold = false;
  for (auto &t : transitions)
  {
    has_link |= (t.first == Phase::LINK);
    has_upload |= (t.first == Phase::MISSION_UPLOAD);
    has_ground_wait |= (t.first == Phase::WAIT_GROUND_START);
    has_preoff |= (t.first == Phase::PRE_OFFBOARD);
    has_cruise |= (t.first == Phase::OFFBOARD_CRUISE);
    has_hold |= (t.first == Phase::HOLD);
  }
  check(has_link && has_upload && has_ground_wait && has_preoff && has_cruise && has_hold,
        "走完上传->等地面站起飞->接管->返回原点悬停");
  check(pilot.holdReason() == HoldReason::MISSION_DONE, "任务完成 -> HOLD（MISSION_DONE）");

  // 上传握手：板载发 44 一次 / 收到飞控 51 = N 次 / 板载发 73 = N 条 / 收到飞控 47 一次
  // 注：fcu.log 只记录飞控"发出"的帧，44/73 是板载发出的，须用飞控侧收包计数
  int c51 = 0, c47 = 0;
  for (auto &s : fcu.log)
  {
    c51 += (s.find("(51)") != std::string::npos);
    c47 += (s.find("(47)") != std::string::npos);
  }
  check(fcu.recv_count44 == 1 && c51 == (int)fcu.mission.size() && c47 == 1,
        "上传握手计数正确 (44=1, 51=N, 73=N, 47=1)");
  std::printf("  [info] 44=%d 51=%d 47=%d, 飞控收到 ITEM_INT=%zu 条\n",
              fcu.recv_count44, c51, c47, fcu.mission.size());
  check(observed_ground_wait && no_early_setpoints, "等待地面站期间不抢占飞控");
  check(fcu.mission.size() == route.size(), "mission = 速度 + 所有 NAV 航点（含返回点）");
  check(fcu.mission.front().command == 178 && fcu.mission.back().command == 16,
        "首条速度，末条普通返回航点");
  bool only_navigation = true;
  for (const auto &item : fcu.mission) only_navigation &= item.command == 178 || item.command == 16;
  check(only_navigation, "mission 中没有起飞/降落/RTL 命令");
  bool mode_only = true;
  for (auto cmd : fcu.received_commands) mode_only &= cmd == CMD_DO_SET_MODE;
  for (auto mode : fcu.requested_modes) mode_only &= mode == MODE_OFFBOARD;
  check(mode_only, "机载只请求 Offboard，不请求解锁/起飞/降落/RTL/AUTO");
  check(fcu.mission.back().frame == 3, "frame = 3 相对高度");
  check(fcu.pos.head<2>().norm() <= cfg.wp_reach_tol && std::abs(fcu.pos.z() - 30.0) <= cfg.wp_reach_tol,
        "完成后位于原点上方 30 m，仍未降落");

  // 锁状态：任务完成后保持悬停，不解锁也不上锁
  check(fcu.armed == true, "任务完成后仍处于解锁状态（悬停等人工接管）");
  // 航点进度同步
  check(max_leg == (int)route.size() - 1, "推进到最后一段");
  check(set_current_seqs.size() == route.size() - 1, "每个航点都发了 MISSION_SET_CURRENT");
  check(!set_current_seqs.empty() && set_current_seqs.front() == 1, "第一个航点对应 mission seq=1");
  check(!set_current_seqs.empty() && set_current_seqs.back() == (uint16_t)route.size() - 1,
        "最后一个航点对应 mission seq 递增正确");

  // 接管过程：进 Offboard 前已在发 setpoint，且首帧 ≈ 当前位姿
  check(pilot.holding(), "接管结束后停在 HOLD 悬停");
  check(offboard_sp_frames > 200, "Offboard 期间持续发 setpoint");
  std::printf("  [info] setpoint 帧数=%d, 接管首帧位置=(%.2f, %.2f, %.2f)\n",
              offboard_sp_frames, first_sp_enu.x(), first_sp_enu.y(), first_sp_enu.z());
  check(pilot.crcErrors() == 0, "全流程 CRC 无错");

  // 提频：协议 2.5.1（req_stream_id = 报文ID）
  bool r_gpi = false, r_att = false;
  for (auto &r : fcu.rate_log)
  {
    if (r.first == GlobalPositionInt::ID) { r_gpi = true; check(r.second >= 50, "GLOBAL_POSITION_INT(33) 提到 >=50Hz"); }
    if (r.first == Attitude::ID) { r_att = true; check(r.second >= 100, "ATTITUDE(30) 提到 >=100Hz"); }
  }
  check(r_gpi && r_att, "起飞前已发出提频请求");
  std::printf("  [info] 提频请求条数=%zu, 最终 GPI=%uHz ATT=%uHz\n",
              fcu.rate_log.size(), (unsigned)fcu.rate_gpi, (unsigned)fcu.rate_att);

  // A later ground-station LAND mode must withdraw onboard control, not be
  // overridden by another mode command or by continued Offboard setpoints.
  fcu.main_mode = MODE_AUTO;
  fcu.sub_mode = SUB_AUTO_LAND;
  fcu.last_hb = -1e9;
  std::vector<std::vector<uint8_t>> landing_telemetry, after_land;
  fcu.tick(0.01, 500.0, landing_telemetry);
  for (const auto &f : landing_telemetry) pilot.onMavlinkBytes(f.data(), f.size());
  pilot.tick(500.0, after_land);
  bool withdrawn = true;
  for (const auto &f : after_land)
    if (f.size() >= 12 && f[0] == 0xFD)
      withdrawn &= f[7] != 84 && f[7] != 76;
  check(withdrawn, "地面站切降落模式后，机载停止 setpoint 且不反向抢模式");

  std::printf("\n%s (fail=%d)\n", g_fail == 0 ? "全部通过" : "存在失败", g_fail);
  return g_fail == 0 ? 0 : 1;
}
