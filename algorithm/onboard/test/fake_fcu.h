#ifndef _ONBOARD_FAKE_FCU_H_
#define _ONBOARD_FAKE_FCU_H_

// 假飞控：只实现协议里与本流程相关的部分，用来离线跑通
// 「上传普通航点 → 等地面站起飞 → Offboard 航点与返回 → 起点悬停」。
//
// 仅用于离线回归测试，不参与实机主程序编译。
//
// 它按 setpoint 以巡航速度直线逼近目标点，所以板载规划出绕行轨迹时，
// 它也会跟着绕 —— 这是验证避障能不能真正执行的关键。

#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Eigen>

#include <mavlink/fcu_interface.h>
#include <mavlink/fcu_messages.h>
#include <mavlink/mavlink_codec.h>

#include <onboard/offboard_pilot.h>

namespace ego_test
{
  using namespace ego_mav;
  using ego_onboard::GeoWaypoint;

  struct FakeFcu
  {
    uint8_t seq = 0;
    double lat0 = 21.0, lon0 = 113.0;
    Eigen::Vector3d pos = Eigen::Vector3d::Zero(); // 局部 ENU（原点 = home）
    Eigen::Vector3d vel = Eigen::Vector3d::Zero(); // 真实速度（ENU，m/s），用于填充 GPI 的 vx/vy/vz
    bool armed = false;
    uint8_t main_mode = 0, sub_mode = 0;
    double takeoff_alt = 30.0;
    double cruise = 5.0;
    double pos_kp = 3.0; // Offboard 位置环增益（1/s）
    uint32_t boot_ms = 1000;

    // mission 状态
    std::vector<MissionItemInt> mission;
    std::vector<GeoWaypoint> route;
    uint16_t expect_seq = 0;
    uint16_t cur_seq = 0;
    uint16_t set_current_seen = 0xFFFF;

    std::vector<std::string> log;   // 飞控**发出**的报文名
    std::vector<uint32_t> recv_ids; // 飞控**收到**的报文 ID（板载 -> 飞控）

    // setpoint 统计
    Parser parser;
    bool sp_seen = false;
    int sp_count = 0;
    Eigen::Vector3d sp_first_ned = Eigen::Vector3d::Zero();
    Eigen::Vector3d sp_last_ned = Eigen::Vector3d::Zero();
    Eigen::Vector3d sp_vel_ned = Eigen::Vector3d::Zero();
    uint16_t sp_type_mask = 0;
    uint8_t sp_first_coord_frame = 0xFF;
    uint16_t sp_first_type_mask = 0;

    double last_hb = -1e9, last_gpi = -1e9, last_att = -1e9, last_gps = -1e9;
    uint16_t rate_gpi = 2, rate_att = 10;               // 协议默认频率
    uint8_t gps_fix = 6;                                // GPS_RAW_INT.fix_type（6 = RTK 固定解）
    uint8_t gps_sats = 20;
    std::vector<std::pair<uint8_t, uint16_t>> rate_log; // 收到的提频请求
    int recv_count44 = 0;                               // 收到的 MISSION_COUNT 次数
    int recv_count41 = 0;                               // 收到的 MISSION_SET_CURRENT 次数
    bool home_sent = false;
    size_t expected_items = 0;
    bool ground_takeoff = false;
    std::vector<uint16_t> received_commands; // onboard only (GCS is separate)
    std::vector<uint8_t> requested_modes;

    void simulateGroundStationTakeoff()
    {
      ground_takeoff = true;
      armed = true;
      main_mode = MODE_AUTO;
      sub_mode = SUB_AUTO_TAKEOFF;
    }

    void send(const std::string &name, uint32_t id, const uint8_t *pl, size_t len,
              std::vector<std::vector<uint8_t>> &out)
    {
      auto f = pack(1, 1, seq++, id, pl, len, true);
      if (!f.empty())
      {
        out.push_back(std::move(f));
        log.push_back(name);
      }
    }

    void onBytes(const uint8_t *d, size_t n, std::vector<std::vector<uint8_t>> &out)
    {
      MavMessage m;
      for (size_t i = 0; i < n; ++i)
      {
        if (!parser.push(d[i], m))
          continue;
        recv_ids.push_back(m.msgid);
        switch (m.msgid)
        {
        case MissionCount::ID:
        {
          ++recv_count44;
          const uint8_t *p = m.payload.data();
          expected_items = rdU16(p, 0);
          mission.clear();
          expect_seq = 0;
          MissionRequestInt r; r.target_system=245; r.target_component=1;
          r.seq = 0;
          uint8_t rp[8];
          r.encode(rp);
          send("MISSION_REQUEST_INT(51)", MissionRequestInt::ID, rp, MissionRequestInt::LEN, out);
          break;
        }
        case MissionItemInt::ID:
        {
          MissionItemInt it;
          MissionItemInt::decode(m.payload.data(), m.payload.size(), it);
          mission.push_back(it);
          expect_seq = (uint16_t)(it.seq + 1);
          if (mission.size() >= expected_items)
          {
            uint8_t ap[8]{};
            wrU8(ap, 0, 245);
            wrU8(ap, 1, 1);
            wrU8(ap, 2, 0);
            send("MISSION_ACK(47)", MissionAck::ID, ap, MissionAck::LEN, out);
          }
          else
          {
            MissionRequestInt r; r.target_system=245; r.target_component=1;
            r.seq = expect_seq;
            uint8_t rp[8];
            r.encode(rp);
            send("MISSION_REQUEST_INT(51)", MissionRequestInt::ID, rp, MissionRequestInt::LEN, out);
          }
          break;
        }
        case MissionSetCurrent::ID:
        {
          ++recv_count41;
          set_current_seen = rdU16(m.payload.data(), 0);
          cur_seq = set_current_seen;
          break;
        }
        case RequestDataStream::ID:
        {
          const uint8_t *p = m.payload.data();
          const uint16_t hz = rdU16(p, 0);
          const uint8_t msgid = rdU8(p, 4);
          if (msgid == GlobalPositionInt::ID) rate_gpi = hz;
          if (msgid == Attitude::ID) rate_att = hz;
          rate_log.push_back({msgid, hz});
          break;
        }
        case CommandLong::ID:
        {
          const uint8_t *p = m.payload.data();
          const float p1 = rdF32(p, 0), p2 = rdF32(p, 4), p3 = rdF32(p, 8);
          const uint16_t cmd = rdU16(p, 28);
          received_commands.push_back(cmd);
          if (cmd == CMD_COMPONENT_ARM_DISARM)
          {
            armed = (p1 > 0.5f);
            uint8_t ap[8]{};
            wrU16(ap, 0, cmd);
            wrU8(ap, 2, 0);
            send("COMMAND_ACK(77)", CommandAck::ID, ap, CommandAck::LEN, out);
            if (armed)
            {
              main_mode = MODE_AUTO;
              sub_mode = SUB_AUTO_TAKEOFF;
              cur_seq = 0;
            }
          }
          else if (cmd == CMD_DO_SET_MODE)
          {
            requested_modes.push_back((uint8_t)p2);
            main_mode = (uint8_t)p2;
            sub_mode = (uint8_t)p3;
            if (main_mode == MODE_AUTO && sub_mode == SUB_AUTO_MISSION && cur_seq < 2)
              cur_seq = 1; // 从速度命令开始按序执行
            uint8_t ap[8]{};
            wrU16(ap, 0, cmd);
            wrU8(ap, 2, 0);
            send("COMMAND_ACK(77)", CommandAck::ID, ap, CommandAck::LEN, out);
          }
          else if (cmd == CMD_NAV_LAND)
          {
            main_mode = MODE_AUTO;
            sub_mode = SUB_AUTO_LAND;
            uint8_t ap[8]{};
            wrU16(ap, 0, cmd);
            wrU8(ap, 2, 0);
            send("COMMAND_ACK(77)", CommandAck::ID, ap, CommandAck::LEN, out);
          }
          break;
        }
        case SetPositionTargetLocalNed::ID:
        {
          const uint8_t *p = m.payload.data();
          const Eigen::Vector3d ned(rdF32(p, 4), rdF32(p, 8), rdF32(p, 12));
          const Eigen::Vector3d vned(rdF32(p, 16), rdF32(p, 20), rdF32(p, 24));
          if (!sp_seen)
          {
            sp_seen = true;
            sp_first_ned = ned;
            sp_first_coord_frame = rdU8(p, 52);   // 应为 1 = MAV_FRAME_LOCAL_NED
            sp_first_type_mask = rdU16(p, 48);    // 位置 + 速度 + yaw，忽略加速度/yaw_rate
          }
          sp_last_ned = ned;
          sp_vel_ned = vned;
          sp_type_mask = rdU16(p, 48);
          ++sp_count;
          break;
        }
        default:
          break;
        }
      }
    }

    // 周期：更新自身位置 + 产出下行报文
    void tick(double dt, double now, std::vector<std::vector<uint8_t>> &out)
    {
      const Eigen::Vector3d pos_prev = pos;
      // ---- 位置更新 ----
      if (main_mode == MODE_OFFBOARD)
      {
        // 真实位置环：速度前馈 + 位置 P 修正。
        // （早期版本是"朝 setpoint 直线冲"，会把规划出的绕行轨迹切角抄近道，
        //   测出来的撞击是仿真保真度不够，不是规划的问题。）
        const Eigen::Vector3d p_t = nedToEnu(sp_last_ned);
        Eigen::Vector3d v_t = nedToEnu(sp_vel_ned);
        // 遵守 type_mask：bit3=忽略 vx（这里三位一起用时表示"不要速度前馈"）
        if (sp_type_mask & (1 << 3))
          v_t.setZero();
        Eigen::Vector3d v = v_t + pos_kp * (p_t - pos);
        const double vmax = cruise * 1.5;
        if (v.norm() > vmax)
          v = v.normalized() * vmax;
        pos += v * dt;
      }
      else if (main_mode == MODE_AUTO)
      {
        if (sub_mode == SUB_AUTO_TAKEOFF)
        {
          pos.z() = std::min(takeoff_alt, pos.z() + 2.0 * dt);
          if (pos.z() >= takeoff_alt - 0.05)
          {
            main_mode = ground_takeoff ? MODE_POS_HOLD : MODE_AUTO;
            sub_mode = ground_takeoff ? 0 : SUB_AUTO_MISSION;
            cur_seq = ground_takeoff ? 0 : 1;
          }
        }
        else if (sub_mode == SUB_AUTO_MISSION)
        {
          if (cur_seq == 1)
          {
            cur_seq = 2; // 速度命令，立即完成
          }
          else if (cur_seq >= 2 && cur_seq < 2 + (int)route.size() - 1)
          {
            const int k = cur_seq - 1; // mission seq = k+1 <-> route_[k]
            Eigen::Vector3d g = geoToEnu(route[k]);
            Eigen::Vector3d d = g - pos;
            const double lim = cruise * dt;
            if (d.norm() <= lim || d.norm() < 0.3)
            {
              pos = g;
              ++cur_seq;
            }
            else
              pos += d.normalized() * lim;
          }
          else
          {
            sub_mode = SUB_AUTO_RTL;
          }
        }
        else if (sub_mode == SUB_AUTO_RTL)
        {
          Eigen::Vector3d home(0, 0, takeoff_alt);
          Eigen::Vector3d d = home - pos;
          const double lim = cruise * dt;
          if (d.norm() < 1.0)
            pos = home;
          else
            pos += d.normalized() * lim;
        }
        else if (sub_mode == SUB_AUTO_LAND)
        {
          pos.z() = std::max(0.0, pos.z() - 1.5 * dt);
        }
      }

      // ---- 下行报文 ----
      // 真实速度（供 GLOBAL_POSITION_INT 的 vx/vy/vz；板载的航位推算就靠它积分）
      vel = (dt > 1e-9) ? Eigen::Vector3d((pos - pos_prev) / dt) : Eigen::Vector3d::Zero();
      if (now - last_hb >= 1.0)
      {
        last_hb = now;
        Heartbeat hb;
        hb.custom_mode = ((uint32_t)sub_mode << 24) | ((uint32_t)main_mode << 16);
        hb.type = 2; // MAV_TYPE_QUADROTOR
        hb.autopilot = 12;
        hb.base_mode = armed ? 0xC0 : 0x40;
        hb.system_status = 4;
        uint8_t hp[Heartbeat::LEN];
        hb.encode(hp);
        send("HEARTBEAT(0)", Heartbeat::ID, hp, Heartbeat::LEN, out);
      }
      if (!home_sent)
      {
        home_sent = true;
        uint8_t p[HomePosition::LEN];
        std::memset(p, 0, sizeof(p));
        wrI32(p, 0, (int32_t)std::llround(lat0 * 1e7));
        wrI32(p, 4, (int32_t)std::llround(lon0 * 1e7));
        send("HOME_POSITION(242)", HomePosition::ID, p, HomePosition::LEN, out);
      }
      // 位置/姿态频率由提频请求决定（默认 2Hz / 10Hz，协议默认值）
      if (now - last_gpi >= 1.0 / (double)rate_gpi)
      {
        last_gpi = now;
        const double lat0r = lat0 * M_PI / 180.0;
        const double lat = lat0 + pos.y() / GeoOrigin::kR * 180.0 / M_PI;
        const double lon = lon0 + pos.x() / (GeoOrigin::kR * std::cos(lat0r)) * 180.0 / M_PI;
        uint8_t p[GlobalPositionInt::LEN];
        std::memset(p, 0, sizeof(p));
        boot_ms = (uint32_t)(now * 1000.0) + 1000; // 与 ATTITUDE 同一个 boot 时钟
        wrU32(p, 0, boot_ms);
        wrI32(p, 4, (int32_t)std::llround(lat * 1e7));
        wrI32(p, 8, (int32_t)std::llround(lon * 1e7));
        wrI32(p, 16, (int32_t)std::llround(pos.z() * 1000.0));
        // 速度：NED，cm/s（vx 北 = +y，vy 东 = +x，vz 地 = -z）
        wrU16(p, 20, (uint16_t)(int16_t)std::llround(vel.y() * 100.0));
        wrU16(p, 22, (uint16_t)(int16_t)std::llround(vel.x() * 100.0));
        wrU16(p, 24, (uint16_t)(int16_t)std::llround(-vel.z() * 100.0));
        send("GLOBAL_POSITION_INT(33)", GlobalPositionInt::ID, p, GlobalPositionInt::LEN, out);
      }
      if (now - last_att >= 1.0 / (double)rate_att)
      {
        last_att = now;
        uint8_t p[Attitude::LEN];
        std::memset(p, 0, sizeof(p));
        boot_ms = (uint32_t)(now * 1000.0) + 1000;
        wrU32(p, 0, boot_ms);
        send("ATTITUDE(30)", Attitude::ID, p, Attitude::LEN, out);
      }
      // GPS_RAW_INT 固定 2Hz：fix_type 是板载判断"丢星"的唯一依据
      if (now - last_gps >= 0.5)
      {
        last_gps = now;
        const double lat0r = lat0 * M_PI / 180.0;
        const double lat = lat0 + pos.y() / GeoOrigin::kR * 180.0 / M_PI;
        const double lon = lon0 + pos.x() / (GeoOrigin::kR * std::cos(lat0r)) * 180.0 / M_PI;
        uint8_t p[GpsRawInt::LEN];
        std::memset(p, 0, sizeof(p));
        wrI32(p, 8, (int32_t)std::llround(lat * 1e7));
        wrI32(p, 12, (int32_t)std::llround(lon * 1e7));
        wrI32(p, 16, (int32_t)std::llround(pos.z() * 1000.0));
        wrU16(p, 20, gps_fix >= 3 ? 80 : 9999); // eph：cm
        wrU16(p, 22, gps_fix >= 3 ? 120 : 9999);
        wrU8(p, 28, gps_fix);
        wrU8(p, 29, gps_sats);
        send("GPS_RAW_INT(24)", GpsRawInt::ID, p, GpsRawInt::LEN, out);
      }
    }

  private:
    // 经纬高（相对起飞点）-> 局部 ENU
    Eigen::Vector3d geoToEnu(const GeoWaypoint &w) const
    {
      const double lat0r = lat0 * M_PI / 180.0;
      Eigen::Vector3d g;
      g.x() = (w.lon_deg - lon0) * M_PI / 180.0 * std::cos(lat0r) * GeoOrigin::kR;
      g.y() = (w.lat_deg - lat0) * M_PI / 180.0 * GeoOrigin::kR;
      g.z() = w.alt_rel_m;
      return g;
    }
  };

} // namespace ego_test

#endif
