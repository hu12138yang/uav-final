// 飞控 MAVLink 对齐自检（不需要真机）：
//   1) 帧编解码往返 + CRC 拒错
//   2) ENU <-> NED 位置/速度/加速度/yaw 换算
//   3) ATTITUDE(NED/FRD) -> 规划侧(ENU/FLU) 四元数
//   4) 经纬 -> 局部 ENU 平面
//   5) GLOBAL_POSITION_INT -> 位姿（含 2Hz 位置的外推）
//   6) QuadrotorCommand(ENU) -> SET_POSITION_TARGET_LOCAL_NED(NED)
#include <cmath>
#include <cstdio>
#include <vector>

#include <ego_core/trajectory_tracker.h>

#include "mavlink/fcu_interface.h"
#include "mavlink/mavlink_codec.h"

using namespace ego_mav;

namespace
{
  int g_fail = 0;
  void check(bool ok, const char *what)
  {
    std::printf("  [%s] %s\n", ok ? " OK " : "FAIL", what);
    if (!ok) g_fail++;
  }
  bool near(double a, double b, double eps = 1e-6) { return std::fabs(a - b) < eps; }

  std::vector<uint8_t> buildGpi(int32_t lat_e7, int32_t lon_e7, int32_t rel_alt_mm,
                                int16_t vx, int16_t vy, int16_t vz, uint32_t tboot_ms)
  {
    std::vector<uint8_t> p(GlobalPositionInt::LEN, 0);
    wrU32(p.data(), 0, tboot_ms);
    wrI32(p.data(), 4, lat_e7);
    wrI32(p.data(), 8, lon_e7);
    wrI32(p.data(), 12, rel_alt_mm); // alt（海拔）这里不关心
    wrI32(p.data(), 16, rel_alt_mm);
    wrU16(p.data(), 20, (uint16_t)vx);
    wrU16(p.data(), 22, (uint16_t)vy);
    wrU16(p.data(), 24, (uint16_t)vz);
    wrU16(p.data(), 26, 0);
    return p;
  }

  std::vector<uint8_t> buildAttitude(float roll, float pitch, float yaw, uint32_t tboot_ms)
  {
    std::vector<uint8_t> p(Attitude::LEN, 0);
    wrU32(p.data(), 0, tboot_ms);
    wrF32(p.data(), 4, roll);
    wrF32(p.data(), 8, pitch);
    wrF32(p.data(), 12, yaw);
    return p;
  }
} // namespace

int main()
{
  // --------------------------------------------------------- 1. 帧编解码
  std::printf("== 1. 帧编解码（ID=84）+ CRC ==\n");
  {
    SetPositionTargetLocalNed sp;
    sp.time_boot_ms = 123456;
    sp.x = 1.5f; sp.y = -2.5f; sp.z = -3.5f;
    sp.vx = 0.1f; sp.vy = 0.2f; sp.vz = 0.3f;
    sp.afx = 0.01f; sp.afy = 0.02f; sp.afz = 0.03f;
    sp.yaw = 1.234f; sp.yaw_rate = 0.5f;
    sp.type_mask = 0x09C0;
    uint8_t payload[SetPositionTargetLocalNed::LEN];
    sp.encode(payload);

    auto frame = pack(245, 1, 7, SetPositionTargetLocalNed::ID, payload, sizeof(payload), true);
    check(!frame.empty(), "打包成功");
    check(frame.size() == 10 + 53 + 2, "v2 帧长 = 10 + 53 + 2");
    check(frame[0] == 0xFD, "STX = 0xFD (MAVLink2)");

    Parser parser;
    MavMessage msg;
    bool got = false;
    for (uint8_t b : frame)
      if (parser.push(b, msg)) got = true;
    check(got, "解析成功");
    check(msg.msgid == 84, "msgid = 84");
    check(msg.sysid == 245 && msg.compid == 1 && msg.seq == 7, "sysid/compid/seq 透传");
    check(msg.payload.size() == 53, "载荷 53 字节");
    check(parser.crcErrors() == 0, "CRC 无错");

    // 逐字段回读（按文档偏移）
    const uint8_t *p = msg.payload.data();
    check(near(rdF32(p, 4), 1.5, 1e-6) && near(rdF32(p, 8), -2.5, 1e-6) && near(rdF32(p, 12), -3.5, 1e-6),
          "xyz 偏移 4/8/12 正确");
    check(near(rdF32(p, 40), 1.234, 1e-6), "yaw 偏移 40 正确");
    check(rdU16(p, 48) == 0x09C0, "type_mask 偏移 48 正确");
    check(rdU8(p, 50) == 1 && rdU8(p, 51) == 1 && rdU8(p, 52) == 1,
          "target_system/component/coordinate_frame 偏移 50/51/52 正确");

    // CRC 拒错：改一个字节
    auto bad = frame;
    bad[12] ^= 0xFF;
    Parser p2;
    MavMessage m2;
    bool got2 = false;
    for (uint8_t b : bad)
      if (p2.push(b, m2)) got2 = true;
    check(!got2 && p2.crcErrors() > 0, "篡改后 CRC 校验失败");
  }

  // ------------------------------------------------- 2. ENU <-> NED 换算
  std::printf("\n== 2. ENU <-> NED 换算 ==\n");
  {
    const Eigen::Vector3d e(3, 4, 5); // 东3 北4 上5
    const Eigen::Vector3d n = enuToNed(e);
    check(near(n.x(), 4) && near(n.y(), 3) && near(n.z(), -5), "位置: 北=东_enu, 东=北_enu, 下=-上");
    check((nedToEnu(n) - e).norm() < 1e-12, "往返一致");

    check(near(yawEnuToNed(kPi / 2), 0.0), "朝东(ENU yaw=pi/2) -> NED yaw=0");
    check(near(yawEnuToNed(0.0), kPi / 2), "朝北(ENU yaw=0) -> NED yaw=pi/2");
    check(near(yawNedToEnu(yawEnuToNed(0.3)), 0.3), "yaw 往返一致");
    check(near(yawRateEnuToNed(0.7), -0.7), "yaw_rate 取反");
  }

  // --------------------------------- 3. ATTITUDE(NED/FRD) -> ENU/FLU
  std::printf("\n== 3. 姿态换算（关键约定）==\n");
  {
    // NED yaw=0 表示朝北 -> ENU 下机头指向 +y，机体系 x 轴应映射到 (0,1,0)
    {
      const Eigen::Quaterniond q = attitudeNedToEnuBody(0, 0, 0);
      const Eigen::Vector3d fwd = q * Eigen::Vector3d::UnitX();
      check(near(fwd.x(), 0.0, 1e-9) && near(fwd.y(), 1.0, 1e-9), "NED yaw=0(朝北) -> 机头指向 ENU +y");
    }
    // NED yaw=pi/2 表示朝东 -> ENU 下机头指向 +x
    {
      const Eigen::Quaterniond q = attitudeNedToEnuBody(0, 0, kPi / 2);
      const Eigen::Vector3d fwd = q * Eigen::Vector3d::UnitX();
      check(near(fwd.x(), 1.0, 1e-9) && near(fwd.y(), 0.0, 1e-9), "NED yaw=pi/2(朝东) -> 机头指向 ENU +x");
    }
    // 纯横滚 roll 绕机体系 x 轴：机体系 y(左) 应向上抬
    {
      const Eigen::Quaterniond q = attitudeNedToEnuBody(0.3, 0, 0);
      const Eigen::Vector3d left = q * Eigen::Vector3d::UnitY();
      check(left.z() > 0.1, "正横滚(roll>0) -> 机体左方向抬起（FLU 约定一致）");
    }
  }

  // ------------------------------------------------ 4. 经纬 -> ENU 平面
  std::printf("\n== 4. 经纬 -> 局部 ENU ==\n");
  {
    GeoOrigin o;
    o.lat_deg = 21.0;
    o.lon_deg = 113.0;
    o.valid = true;
    const Eigen::Vector2d xy = geoToEnuXY(21.001, 113.0, o); // 正北 0.001 度
    check(near(xy.x(), 0.0, 1e-6), "东向 ≈ 0");
    check(std::fabs(xy.y() - 111.32) < 0.1, "北向 ≈ 111.32 m");
  }

  // -------------------------------- 5. GLOBAL_POSITION_INT -> 位姿
  std::printf("\n== 5. 位置流 -> 位姿（含外推）==\n");
  {
    GeoOrigin o;
    o.lat_deg = 21.0; o.lon_deg = 113.0; o.valid = true;

    PoseAssembler pa;
    pa.setOrigin(o);

    GlobalPositionInt g;
    auto pl = buildGpi(210010000, 1130000000, 5000 /*5 m 相对高*/, 200 /*北 2m/s*/, 0, 0, 1000);
    check(GlobalPositionInt::decode(pl.data(), pl.size(), g), "GPI 解码成功");
    pa.onGlobalPositionInt(g);

    Attitude a;
    auto al = buildAttitude(0, 0, 0, 1000);
    check(Attitude::decode(al.data(), al.size(), a), "ATTITUDE 解码成功");

    ego_sensor::TimedPose tp;
    check(pa.onAttitude(a, 1.0, tp), "组装位姿成功");
    check(std::fabs(tp.p.z() - 5.0) < 1e-9, "z 用 relative_alt = 5 m");
    check(std::fabs(tp.p.y() - 111.32) < 0.1, "y(北) ≈ 111.32 m");

    // 0.2 s 后再来一条姿态：位置应外推 +0.4 m（北向 2 m/s）
    check(pa.onAttitude(a, 1.2, tp), "再次组装");
    check(std::fabs(tp.p.y() - (111.32 + 0.4)) < 0.15, "2Hz 位置外推 0.2s ≈ +0.4 m");
  }

  // --------------------- 6. QuadrotorCommand(ENU) -> SETPOINT(NED)
  std::printf("\n== 6. 指令 -> SET_POSITION_TARGET_LOCAL_NED ==\n");
  {
    ego_planner::QuadrotorCommand cmd;
    cmd.position = Eigen::Vector3d(3, 4, 5);      // ENU
    cmd.velocity = Eigen::Vector3d(1, 0, 0);      // 向东 1 m/s
    cmd.acceleration = Eigen::Vector3d(0, 0.5, 0); // 向北 0.5
    cmd.yaw = kPi / 2;                            // 朝东
    cmd.yaw_dot = 0.4;
    cmd.trajectory_id = 12;

    SetpointConfig cfg;
    SetPositionTargetLocalNed sp;
    toSetpoint(cmd, 22345, cfg, sp);

    check(near(sp.x, 4) && near(sp.y, 3) && near(sp.z, -5), "位置 NED 正确");
    check(near(sp.vx, 0) && near(sp.vy, 1) && near(sp.vz, 0), "速度 NED 正确");
    // 加速度是 ENU 的"北向 0.5" -> NED 的 x(北) = afx
    check(near(sp.afx, 0.5, 1e-6) && near(sp.afy, 0.0, 1e-6), "加速度 NED 正确");
    check(near(sp.yaw, 0.0, 1e-9), "朝东的 NED yaw = 0");
    check(near(sp.yaw_rate, -0.4, 1e-6), "yaw_rate 取反");
    check(sp.type_mask == cfg.type_mask && sp.coordinate_frame == 1 && sp.time_boot_ms == 22345,
          "type_mask/frame/时间戳 正确");
  }

  // --------------- 7. MAVLink2 截断载荷（末尾零被裁掉）必须仍能解析
  std::printf("\n== 7. MAVLink2 截断载荷 ==\n");
  {
    // 28 字节 GPI：vz(24) 与 hdg(26) 都是 0 -> V2 会裁掉最后 4 字节，实际只有 24
    auto pl = buildGpi(210010000, 1130000000, 5000, 200, 0, 0, 1000);
    pl.resize(24);
    GlobalPositionInt g;
    check(GlobalPositionInt::decode(pl.data(), pl.size(), g), "截断载荷解码成功（不再被拒）");
    check(g.lat == 210010000 && g.relative_alt == 5000, "已发送字段正确");
    check(g.vz == 0 && g.hdg == 0, "被裁掉的字段补 0");
  }

  // --------------------------------- 8. CRC_EXTRA 覆盖入口（台架定性用）
  std::printf("\n== 8. CRC_EXTRA 可覆盖 ==\n");
  {
    uint8_t ex = 0;
    crcExtra(242, ex);
    check(ex == 36, "默认取厂商 Python 的值 (HOME_POSITION=36)");
    setCrcExtra(242, 104); // 换官方值
    crcExtra(242, ex);
    check(ex == 104, "setCrcExtra 覆盖生效");
    setCrcExtra(242, 36); // 还原
    crcExtra(242, ex);
    check(ex == 36, "还原成功");
  }

  // ------------------- 9. 任务上传 + 命令/模式（协议 2.6.1 / 2.4.x / 2.5.2）
  std::printf("\n== 9. 任务上传与命令/模式 ==\n");
  {
    // 一份典型巡检航线：起飞(22) -> 速度(178) -> 航点(16)x2 -> 返航(20)
    std::vector<MissionItemInt> mission;
    mission.push_back(MissionItemInt::takeoff(0, 21.0000, 113.0000, 30.0));
    mission.push_back(MissionItemInt::speed(1, 5.0f));
    mission.push_back(MissionItemInt::waypoint(2, 21.0050, 113.0000, 30.0));
    mission.push_back(MissionItemInt::waypoint(3, 21.0100, 113.0000, 30.0));
    mission.push_back(MissionItemInt::rtl(4));

    check(mission[0].command == 22 && mission[4].command == 20, "首条=起飞(22)、末条=返航(20)");
    check(mission[1].command == 178 && mission[1].param1 == 1.f, "速度命令 param1=地速");

    // MISSION_COUNT -> 逐条 MISSION_ITEM_INT，全程走真实帧
    MissionCount mc;
    mc.count = (uint16_t)mission.size();
    uint8_t pl[8];
    mc.encode(pl);
    auto f0 = pack(245, 1, 0, MissionCount::ID, pl, MissionCount::LEN, true);
    Parser pm;
    MavMessage mm;
    bool ok0 = false;
    for (uint8_t b : f0)
      if (pm.push(b, mm)) ok0 = true;
    check(ok0 && mm.msgid == 44, "MISSION_COUNT 上线并可解析");

    // 飞控要第 2 条 -> 我们回 MISSION_ITEM_INT(2)
    MissionRequestInt req;
    uint8_t rp[8];
    wrU16(rp, 0, 2); wrU8(rp, 2, 245); wrU8(rp, 3, 1); wrU8(rp, 4, 0);
    check(MissionRequestInt::decode(rp, MissionRequestInt::LEN, req) && req.seq == 2, "MISSION_REQUEST_INT seq=2");

    uint8_t ip[MissionItemInt::LEN];
    mission[req.seq].encode(ip);
    check(rdU16(ip, 28) == 2 && rdU16(ip, 30) == 16, "ITEM_INT 偏移 28/30 = seq/command");
    check(rdI32(ip, 16) == 210050000, "x = 纬度 degE7");
    check(rdI32(ip, 20) == 1130000000, "y = 经度 degE7");
    check(near(rdF32(ip, 24), 30.0, 1e-5), "z = 相对高度 30 m");
    check(rdU8(ip, 34) == 3, "frame = GLOBAL_RELATIVE_ALT(3) → 高度是相对高度");

    auto fa = pack(245, 1, 0, 73, ip, MissionItemInt::LEN, true);
    Parser pa2;
    bool got = false;
    for (uint8_t b : fa)
      if (pa2.push(b, mm)) got = true;
    MissionItemInt back;
    check(got && MissionItemInt::decode(mm.payload.data(), mm.payload.size(), back), "ITEM_INT 帧往返");
    check(back.seq == 2 && back.command == 16 && back.frame == 3, "往返后 seq/command/frame 一致");

    // 模式：巡航 = 自动(4)/自动任务(4)；接管 = Offboard(6)
    CommandLong mc2 = CommandLong::setMode(MODE_AUTO, SUB_AUTO_MISSION);
    check(mc2.command == 176 && mc2.param1 == 1.f && mc2.param2 == 4.f && mc2.param3 == 4.f,
          "切巡航: command=176 param2=4 param3=4");
    CommandLong mo = CommandLong::setMode(MODE_OFFBOARD, 0);
    check(mo.param2 == 6.f && mo.param3 == 0.f, "切接管: param2=6 (Offboard)");

    // HEARTBEAT 模式/解锁解析
    Heartbeat hb;
    hb.custom_mode = ((uint32_t)SUB_AUTO_MISSION << 24) | ((uint32_t)MODE_AUTO << 16);
    hb.base_mode = 192 | 0x80; // 已解锁
    uint8_t hbp[Heartbeat::LEN];
    hb.encode(hbp);
    Heartbeat hb2;
    check(Heartbeat::decode(hbp, Heartbeat::LEN, hb2), "HEARTBEAT 解码");
    check(hb2.mainMode() == 4 && hb2.subMode() == 4, "custom_mode 主模式=4 细分=4");
    check(hb2.armed(), "(base_mode&0x80) 判定已解锁");
  }

  std::printf("\n%s (fail=%d)\n", g_fail == 0 ? "全部通过" : "存在失败", g_fail);
  return g_fail == 0 ? 0 : 1;
}
