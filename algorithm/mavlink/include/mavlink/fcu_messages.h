#ifndef _MAVLINK_FCU_MESSAGES_H_
#define _MAVLINK_FCU_MESSAGES_H_

// 飞控 MAVLink 报文的编解码。
// 字段偏移、长度、CRC_EXTRA 全部按《飞控地面站交互协议 V2.8.4》里的表逐条对齐，
// 不依赖任何 MAVLink 库：文档已经给了精确偏移量，直接按偏移读写最不容易错。
//
// 字节序：MAVLink 全部小端。

#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

namespace ego_mav
{

  // ---------------------------------------------------------------- 基础读写
  inline void wrU8(uint8_t *p, size_t off, uint8_t v) { p[off] = v; }
  inline void wrU16(uint8_t *p, size_t off, uint16_t v)
  {
    p[off] = (uint8_t)(v & 0xFF);
    p[off + 1] = (uint8_t)((v >> 8) & 0xFF);
  }
  inline void wrU32(uint8_t *p, size_t off, uint32_t v)
  {
    for (int i = 0; i < 4; ++i) p[off + i] = (uint8_t)((v >> (8 * i)) & 0xFF);
  }
  inline void wrI32(uint8_t *p, size_t off, int32_t v) { wrU32(p, off, (uint32_t)v); }
  inline void wrU64(uint8_t *p, size_t off, uint64_t v)
  {
    for (int i = 0; i < 8; ++i) p[off + i] = (uint8_t)((v >> (8 * i)) & 0xFF);
  }
  inline void wrF32(uint8_t *p, size_t off, float v)
  {
    uint32_t u;
    std::memcpy(&u, &v, 4);
    wrU32(p, off, u);
  }

  inline uint8_t rdU8(const uint8_t *p, size_t off) { return p[off]; }

  // MAVLink2 会把载荷末尾的 0 字节截断（协议文档："载荷末尾0字节部分做了截断"），
  // 所以解析前必须补零到标准长度；v1 不会截断。厂商 Python 参考实现里的 _pad() 就是这个。
  // 注意：部分报文在 V2 下比 V1 更长（扩展字段），此时 len > full_len，只读前 full_len 字节即可。
  inline void padPayload(const uint8_t *p, size_t len, uint8_t *out, size_t full_len)
  {
    const size_t n = len < full_len ? len : full_len;
    if (n)
      std::memcpy(out, p, n);
    if (full_len > n)
      std::memset(out + n, 0, full_len - n);
  }
  inline uint16_t rdU16(const uint8_t *p, size_t off)
  {
    return (uint16_t)(p[off] | ((uint16_t)p[off + 1] << 8));
  }
  inline uint32_t rdU32(const uint8_t *p, size_t off)
  {
    uint32_t v = 0;
    for (int i = 0; i < 4; ++i) v |= (uint32_t)p[off + i] << (8 * i);
    return v;
  }
  inline int32_t rdI32(const uint8_t *p, size_t off) { return (int32_t)rdU32(p, off); }
  inline uint64_t rdU64(const uint8_t *p, size_t off)
  {
    uint64_t v = 0;
    for (int i = 0; i < 8; ++i) v |= (uint64_t)p[off + i] << (8 * i);
    return v;
  }
  inline float rdF32(const uint8_t *p, size_t off)
  {
    const uint32_t u = rdU32(p, off);
    float f;
    std::memcpy(&f, &u, 4);
    return f;
  }

  // ------------------------------------------------- 输出：板载计算机 -> 飞控
  // ID=84 SET_POSITION_TARGET_LOCAL_NED，载荷 53 字节，CRC_EXTRA=143
  // 文档给的是**最终线序偏移**（MAVLink 会按类型长度重排字段），所以直接按偏移写。
  struct SetPositionTargetLocalNed
  {
    static constexpr uint32_t ID = 84;
    static constexpr size_t LEN = 53;
    static constexpr uint8_t CRC_EXTRA = 143;

    uint32_t time_boot_ms = 0;
    float x = 0, y = 0, z = 0;          // NED 位置(m)：前 / 右 / 下
    float vx = 0, vy = 0, vz = 0;       // NED 速度(m/s)
    float afx = 0, afy = 0, afz = 0;    // NED 加速度(m/s^2)
    float yaw = 0, yaw_rate = 0;        // rad, rad/s
    uint16_t type_mask = 0;
    uint8_t target_system = 1;
    uint8_t target_component = 1;
    uint8_t coordinate_frame = 1;       // MAV_FRAME_LOCAL_NED（原点=起飞点）

    void encode(uint8_t *p) const
    {
      std::memset(p, 0, LEN);
      wrU32(p, 0, time_boot_ms);
      wrF32(p, 4, x);
      wrF32(p, 8, y);
      wrF32(p, 12, z);
      wrF32(p, 16, vx);
      wrF32(p, 20, vy);
      wrF32(p, 24, vz);
      wrF32(p, 28, afx);
      wrF32(p, 32, afy);
      wrF32(p, 36, afz);
      wrF32(p, 40, yaw);
      wrF32(p, 44, yaw_rate);
      wrU16(p, 48, type_mask);
      wrU8(p, 50, target_system);
      wrU8(p, 51, target_component);
      wrU8(p, 52, coordinate_frame);
    }
  };

  // ------------------------------------------------- 输入：飞控 -> 板载计算机
  // ID=33 GLOBAL_POSITION_INT，载荷 28，CRC_EXTRA=104
  struct GlobalPositionInt
  {
    static constexpr uint32_t ID = 33;
    static constexpr size_t LEN = 28;
    static constexpr uint8_t CRC_EXTRA = 104;

    uint32_t time_boot_ms = 0;
    int32_t lat = 0, lon = 0;      // degE7
    int32_t alt = 0;               // 海拔高度 mm
    int32_t relative_alt = 0;      // **相对起飞点**高度 mm  ← 与"定高"语义一致
    int16_t vx = 0, vy = 0, vz = 0; // 北/东/地 cm/s
    uint16_t hdg = 0;              // 航向 0-360 度

    static bool decode(const uint8_t *p, size_t len, GlobalPositionInt &o)
    {
      if (len == 0) return false;
      uint8_t buf[LEN];
      padPayload(p, len, buf, LEN);
      p = buf; // MAVLink2 会截断末尾零，补零后再按固定偏移读
      o.time_boot_ms = rdU32(p, 0);
      o.lat = rdI32(p, 4);
      o.lon = rdI32(p, 8);
      o.alt = rdI32(p, 12);
      o.relative_alt = rdI32(p, 16);
      o.vx = (int16_t)rdU16(p, 20);
      o.vy = (int16_t)rdU16(p, 22);
      o.vz = (int16_t)rdU16(p, 24);
      o.hdg = rdU16(p, 26);
      return true;
    }
  };

  // ID=24 GPS_RAW_INT，载荷 52，CRC_EXTRA=24
  // 只取定位健康相关的字段：fix_type 是判断"丢星"的唯一可靠依据。
  // 0/1=无定位, 2=2D, 3=3D, 4=DGPS, 5=RTK浮动解, 6=RTK固定解, 7=静态固定解
  struct GpsRawInt
  {
    static constexpr uint32_t ID = 24;
    static constexpr size_t LEN = 52;
    static constexpr uint8_t CRC_EXTRA = 24;

    int32_t lat = 0, lon = 0, alt = 0; // degE7 / mm
    uint16_t eph = 9999, epv = 9999;   // 水平 / 垂直精度，cm
    uint8_t fix_type = 0;
    uint8_t satellites_visible = 0;

    static bool decode(const uint8_t *p, size_t len, GpsRawInt &o)
    {
      if (len == 0) return false;
      uint8_t buf[LEN];
      padPayload(p, len, buf, LEN);
      p = buf;
      // 偏移 0 是 time_usec(u64)，这里不用
      o.lat = rdI32(p, 8);
      o.lon = rdI32(p, 12);
      o.alt = rdI32(p, 16);
      o.eph = rdU16(p, 20);
      o.epv = rdU16(p, 22);
      o.fix_type = rdU8(p, 28);
      o.satellites_visible = rdU8(p, 29);
      return true;
    }
  };

  // ID=30 ATTITUDE，载荷 28，CRC_EXTRA=39
  struct Attitude
  {
    static constexpr uint32_t ID = 30;
    static constexpr size_t LEN = 28;
    static constexpr uint8_t CRC_EXTRA = 39;

    uint32_t time_boot_ms = 0;
    float roll = 0, pitch = 0, yaw = 0;             // rad
    float rollspeed = 0, pitchspeed = 0, yawspeed = 0;

    static bool decode(const uint8_t *p, size_t len, Attitude &o)
    {
      if (len == 0) return false;
      uint8_t buf[LEN];
      padPayload(p, len, buf, LEN);
      p = buf;
      o.time_boot_ms = rdU32(p, 0);
      o.roll = rdF32(p, 4);
      o.pitch = rdF32(p, 8);
      o.yaw = rdF32(p, 12);
      o.rollspeed = rdF32(p, 16);
      o.pitchspeed = rdF32(p, 20);
      o.yawspeed = rdF32(p, 24);
      return true;
    }
  };

  // ID=242 HOME_POSITION，载荷 60，CRC_EXTRA=104（官方值；文档未列）
  struct HomePosition
  {
    static constexpr uint32_t ID = 242;
    static constexpr size_t LEN = 60;
    static constexpr uint8_t CRC_EXTRA = 104;

    int32_t latitude = 0, longitude = 0; // degE7
    int32_t altitude = 0;                // mm（海拔）

    static bool decode(const uint8_t *p, size_t len, HomePosition &o)
    {
      if (len == 0) return false;
      uint8_t buf[LEN];
      padPayload(p, len, buf, LEN);
      p = buf;
      o.latitude = rdI32(p, 0);
      o.longitude = rdI32(p, 4);
      o.altitude = rdI32(p, 8);
      return true;
    }
  };

  // ID=2 SYSTEM_TIME，载荷 12，CRC_EXTRA=137（官方值；文档未列）—— 用于时间同步
  struct SystemTime
  {
    static constexpr uint32_t ID = 2;
    static constexpr size_t LEN = 12;
    static constexpr uint8_t CRC_EXTRA = 137;

    uint64_t time_unix_usec = 0;
    uint32_t time_boot_ms = 0;

    static bool decode(const uint8_t *p, size_t len, SystemTime &o)
    {
      if (len == 0) return false;
      uint8_t buf[LEN];
      padPayload(p, len, buf, LEN);
      p = buf;
      o.time_unix_usec = rdU64(p, 0);
      o.time_boot_ms = rdU32(p, 8);
      return true;
    }
  };

  // ID=330 OBSTACLE_DISTANCE，载荷 167，CRC_EXTRA=23（官方值）—— 飞控侧近距兜底
  struct ObstacleDistance
  {
    static constexpr uint32_t ID = 330;
    static constexpr size_t LEN = 167;
    static constexpr uint8_t CRC_EXTRA = 23;

    uint64_t time_usec = 0;
    uint16_t distances[72] = {0}; // cm，65535 = 无效；前方=distances[0]，顺时针；定高=distances[32]

    static bool decode(const uint8_t *p, size_t len, ObstacleDistance &o)
    {
      if (len == 0) return false;
      uint8_t buf[LEN];
      padPayload(p, len, buf, LEN);
      p = buf;
      o.time_usec = rdU64(p, 0);
      for (int i = 0; i < 72; ++i) o.distances[i] = rdU16(p, 8 + 2 * i);
      return true;
    }
  };

  // ID=77 COMMAND_ACK，载荷 10，CRC_EXTRA=143
  struct CommandAck
  {
    static constexpr uint32_t ID = 77;
    static constexpr size_t LEN = 10;
    static constexpr uint8_t CRC_EXTRA = 143;

    uint16_t command = 0;
    uint8_t result = 0;

    static bool decode(const uint8_t *p, size_t len, CommandAck &o)
    {
      if (len == 0) return false;
      uint8_t buf[LEN];
      padPayload(p, len, buf, LEN);
      p = buf;
      o.command = rdU16(p, 0);
      o.result = rdU8(p, 2);
      return true;
    }
  };

  // ------------------------------------------------- 任务/命令相关常量
  // 飞行主模式（COMMAND_LONG command=176 的 param2；HEARTBEAT.custom_mode 也会反馈）
  enum MainMode
  {
    MODE_ALT_HOLD = 2, // 定高
    MODE_POS_HOLD = 3, // 定点
    MODE_AUTO = 4,     // 自动（任务/返航/降落都在这一档的细分里）
    MODE_OFFBOARD = 6, // Offboard ← 我们接管用的
  };
  // 自动模式细分（command=176 的 param3）
  enum AutoSubmode
  {
    SUB_AUTO_TAKEOFF = 2, // 只反馈，不可设置
    SUB_AUTO_TRACK = 3,
    SUB_AUTO_MISSION = 4, // 自动任务 ← 巡航
    SUB_AUTO_RTL = 5,     // 自动返航
    SUB_AUTO_LAND = 6,    // 自动降落
  };
  enum PosHoldSubmode
  {
    SUB_POSHOLD_NORMAL = 0,
    SUB_POSHOLD_AVOID = 2, // 定点避障
  };

  // COMMAND_LONG 的 command 字段
  enum MavCmd
  {
    CMD_NAV_WAYPOINT = 16,
    CMD_NAV_RETURN_TO_LAUNCH = 20,
    CMD_NAV_LAND = 21,
    CMD_NAV_TAKEOFF = 22,      // 任务首条（param7 = 相对/海拔高度）
    CMD_NAV_TAKEOFF_ALT_REL = 24, // 一键起飞：param7 = 相对起飞高度
    CMD_NAV_DELAY = 112,       // 航点停留（param1 = 秒）
    CMD_DO_SET_MODE = 176,
    CMD_DO_SET_SPEED = 178,
    CMD_DO_SET_HOME = 179, // 2.4.11 设置返航点
    CMD_COMPONENT_ARM_DISARM = 400,
  };

  // 坐标系（MISSION_ITEM_INT.frame）
  enum MavFrame
  {
    FRAME_GLOBAL = 0,                // 海拔高
    FRAME_MISSION = 2,               // DO_ 类命令
    FRAME_GLOBAL_RELATIVE_ALT = 3,   // 相对起飞点高度 ← 航线用这个
  };

  // ID=0 HEARTBEAT，载荷 9，CRC_EXTRA=50（双向）
  struct Timesync {
    static constexpr uint32_t ID = 111;
    static constexpr size_t LEN = 18;
    int64_t tc1 = 0, ts1 = 0;
    uint8_t target_system = 0, target_component = 0;
    void encode(uint8_t *p) const {
      wrU64(p, 0, (uint64_t)tc1); wrU64(p, 8, (uint64_t)ts1);
      p[16] = target_system; p[17] = target_component;
    }
    static bool decode(const uint8_t *p, size_t len, Timesync &m) {
      if (!len) return false;
      uint8_t b[LEN]; padPayload(p, len, b, LEN);
      m.tc1 = (int64_t)rdU64(b, 0); m.ts1 = (int64_t)rdU64(b, 8);
      m.target_system = b[16]; m.target_component = b[17]; return true;
    }
  };

  struct Heartbeat
  {
    static constexpr uint32_t ID = 0;
    static constexpr size_t LEN = 9;
    static constexpr uint8_t CRC_EXTRA = 50;

    uint32_t custom_mode = 0;
    uint8_t type = 6;             // 我们发 MAV_TYPE_GCS
    uint8_t autopilot = 8;        // MAV_AUTOPILOT_INVALID
    uint8_t base_mode = 192;      // MAV_MODE_MANUAL_ARMED
    uint8_t system_status = 4;    // MAV_STATE_ACTIVE
    uint8_t mavlink_version = 2;

    bool armed() const { return (base_mode & 0x80) == 0x80; }
    // custom_mode 第 3 字节=主模式，第 4 字节=细分模式（协议 2.5.2）
    uint8_t mainMode() const { return (uint8_t)((custom_mode >> 16) & 0xFF); }
    uint8_t subMode() const { return (uint8_t)((custom_mode >> 24) & 0xFF); }

    void encode(uint8_t *p) const
    {
      std::memset(p, 0, LEN);
      wrU32(p, 0, custom_mode);
      wrU8(p, 4, type);
      wrU8(p, 5, autopilot);
      wrU8(p, 6, base_mode);
      wrU8(p, 7, system_status);
      wrU8(p, 8, mavlink_version);
    }
    static bool decode(const uint8_t *p, size_t len, Heartbeat &o)
    {
      if (len == 0) return false;
      uint8_t buf[LEN];
      padPayload(p, len, buf, LEN);
      p = buf;
      o.custom_mode = rdU32(p, 0);
      o.type = rdU8(p, 4);
      o.autopilot = rdU8(p, 5);
      o.base_mode = rdU8(p, 6);
      o.system_status = rdU8(p, 7);
      o.mavlink_version = rdU8(p, 8);
      return true;
    }
  };

  // ID=76 COMMAND_LONG，载荷 33，CRC_EXTRA=152
  struct CommandLong
  {
    static constexpr uint32_t ID = 76;
    static constexpr size_t LEN = 33;
    static constexpr uint8_t CRC_EXTRA = 152;

    float param1 = 0, param2 = 0, param3 = 0, param4 = 0, param5 = 0, param6 = 0, param7 = 0;
    uint16_t command = 0;
    uint8_t target_system = 1;
    uint8_t target_component = 1;
    uint8_t confirmation = 0;

    void encode(uint8_t *p) const
    {
      std::memset(p, 0, LEN);
      wrF32(p, 0, param1); wrF32(p, 4, param2); wrF32(p, 8, param3);
      wrF32(p, 12, param4); wrF32(p, 16, param5); wrF32(p, 20, param6); wrF32(p, 24, param7);
      wrU16(p, 28, command);
      wrU8(p, 30, target_system);
      wrU8(p, 31, target_component);
      wrU8(p, 32, confirmation);
    }

    // ---- 协议 2.4.x 的几条现成命令 ----
    static CommandLong arm(bool on, uint8_t sys = 1)
    {
      CommandLong c; c.command = CMD_COMPONENT_ARM_DISARM; c.param1 = on ? 1.f : 0.f; c.target_system = sys; return c;
    }
    static CommandLong takeoffRelative(double alt_m, uint8_t sys = 1)
    {
      CommandLong c; c.command = CMD_NAV_TAKEOFF_ALT_REL; c.param7 = (float)alt_m; c.target_system = sys; return c;
    }
    static CommandLong setMode(uint8_t main_mode, uint8_t sub_mode, uint8_t sys = 1)
    {
      CommandLong c; c.command = CMD_DO_SET_MODE;
      c.param1 = 1.f; c.param2 = (float)main_mode; c.param3 = (float)sub_mode; c.target_system = sys; return c;
    }
    static CommandLong rtl(uint8_t sys = 1) { CommandLong c; c.command = CMD_NAV_RETURN_TO_LAUNCH; c.target_system = sys; return c; }
    static CommandLong land(double rel_alt_m, double yaw_deg, uint8_t sys = 1)
    {
      CommandLong c; c.command = CMD_NAV_LAND;
      c.param4 = (float)yaw_deg; c.param5 = 1000; c.param6 = 1000; c.param7 = (float)rel_alt_m;
      c.target_system = sys; return c;
    }
  };

  // ID=66 REQUEST_DATA_STREAM，载荷 6，CRC_EXTRA=148
  // ⚠️ 协议 2.5.1：req_stream_id 填的是**报文ID**（不是传统 MAVLink 的 stream 分组），
  //    所以可以逐条报文改频率。"所有报文由飞机主动发送，可更改频率"。
  // ⚠️ 该字段是 uint8_t → 只能寻址 ID ≤ 255 的报文（如 330 OBSTACLE_DISTANCE 用不了）。
  struct RequestDataStream
  {
    static constexpr uint32_t ID = 66;
    static constexpr size_t LEN = 6;
    static constexpr uint8_t CRC_EXTRA = 148;

    uint16_t req_message_rate = 50;
    uint8_t target_system = 1;
    uint8_t target_component = 1;
    uint8_t req_stream_id = 0;  // = 报文ID
    uint8_t start_stop = 1;

    // 针对某条报文提频（协议 2.5.1）
    static RequestDataStream forMessage(uint8_t msgid, uint16_t hz, uint8_t sys = 1)
    {
      RequestDataStream r;
      r.req_stream_id = msgid;
      r.req_message_rate = hz;
      r.target_system = sys;
      return r;
    }

    void encode(uint8_t *p) const
    {
      std::memset(p, 0, LEN);
      wrU16(p, 0, req_message_rate);
      wrU8(p, 2, target_system);
      wrU8(p, 3, target_component);
      wrU8(p, 4, req_stream_id);
      wrU8(p, 5, start_stop);
    }
  };

  // ---------------------------------------------- 任务上传/下载（协议 2.6）
  // ID=43 MISSION_REQUEST_LIST（请求下载任务列表），V2 载荷 3，CRC_EXTRA=132
  struct MissionRequestList
  {
    static constexpr uint32_t ID = 43;
    static constexpr size_t LEN = 3;
    static constexpr uint8_t CRC_EXTRA = 132;

    uint8_t target_system = 1, target_component = 1;
    uint8_t mission_type = 0; // 0=任务航线, 1=电子围栏

    void encode(uint8_t *p) const
    {
      std::memset(p, 0, LEN);
      wrU8(p, 0, target_system);
      wrU8(p, 1, target_component);
      wrU8(p, 2, mission_type);
    }
    static bool decode(const uint8_t *p, size_t len, MissionRequestList &o)
    {
      if (len == 0) return false;
      uint8_t buf[LEN];
      padPayload(p, len, buf, LEN);
      p = buf;
      o.target_system = rdU8(p, 0);
      o.target_component = rdU8(p, 1);
      o.mission_type = rdU8(p, 2);
      return true;
    }
  };

  // ID=44 MISSION_COUNT，V2 载荷 5（V1 无 mission_type），CRC_EXTRA=221
  struct MissionCount
  {
    static constexpr uint32_t ID = 44;
    static constexpr size_t LEN = 5;
    static constexpr uint8_t CRC_EXTRA = 221;

    uint16_t count = 0;
    uint8_t target_system = 1, target_component = 1;
    uint8_t mission_type = 0; // 0=任务航线, 1=电子围栏

    void encode(uint8_t *p) const
    {
      std::memset(p, 0, LEN);
      wrU16(p, 0, count);
      wrU8(p, 2, target_system);
      wrU8(p, 3, target_component);
      wrU8(p, 4, mission_type);
    }
    static bool decode(const uint8_t *p, size_t len, MissionCount &o)
    {
      if (len == 0) return false;
      uint8_t buf[LEN];
      padPayload(p, len, buf, LEN);
      p = buf;
      o.count = rdU16(p, 0);
      o.target_system = rdU8(p, 2);
      o.target_component = rdU8(p, 3);
      o.mission_type = rdU8(p, 4);
      return true;
    }
  };

  // ID=51 MISSION_REQUEST_INT（飞控来要第 seq 条），载荷 5，CRC_EXTRA=196
  struct MissionRequestInt
  {
    static constexpr uint32_t ID = 51;
    static constexpr size_t LEN = 5;
    static constexpr uint8_t CRC_EXTRA = 196;

    uint16_t seq = 0;
    uint8_t target_system = 1, target_component = 1, mission_type = 0;

    void encode(uint8_t *p) const
    {
      std::memset(p, 0, LEN);
      wrU16(p, 0, seq);
      wrU8(p, 2, target_system);
      wrU8(p, 3, target_component);
      wrU8(p, 4, mission_type);
    }
    static bool decode(const uint8_t *p, size_t len, MissionRequestInt &o)
    {
      if (len == 0) return false;
      uint8_t buf[LEN];
      padPayload(p, len, buf, LEN);
      p = buf;
      o.seq = rdU16(p, 0);
      o.target_system = rdU8(p, 2);
      o.target_component = rdU8(p, 3);
      o.mission_type = rdU8(p, 4);
      return true;
    }
  };

  // ID=47 MISSION_ACK（飞控回执），载荷 3，CRC_EXTRA=153
  struct MissionAck
  {
    static constexpr uint32_t ID = 47;
    static constexpr size_t LEN = 4;
    static constexpr uint8_t CRC_EXTRA = 153;

    uint8_t target_system = 1, target_component = 1;
    uint8_t mission_type = 0;
    uint8_t type = 0; // 0 = MAV_MISSION_ACCEPTED
    bool accepted() const { return type == 0; }

    void encode(uint8_t *p) const
    {
      std::memset(p, 0, LEN);
      wrU8(p, 0, target_system);
      wrU8(p, 1, target_component);
      wrU8(p, 2, type);
      wrU8(p, 3, mission_type);
    }
    static bool decode(const uint8_t *p, size_t len, MissionAck &o)
    {
      if (len == 0) return false;
      uint8_t buf[LEN];
      padPayload(p, len, buf, LEN);
      p = buf;
      o.target_system = rdU8(p, 0);
      o.target_component = rdU8(p, 1);
      o.type = rdU8(p, 2);
      o.mission_type = rdU8(p, 3);
      return true;
    }
  };

  // ID=41 MISSION_SET_CURRENT，载荷 4，CRC_EXTRA=132
  // 用途：Offboard 期间由我们推进航线，必须同步飞控的 mission 进度，
  // 否则交还/返航时飞控会从旧断点（起飞点）往回飞。
  struct MissionSetCurrent
  {
    static constexpr uint32_t ID = 41;
    static constexpr size_t LEN = 4;
    static constexpr uint8_t CRC_EXTRA = 132;

    uint16_t seq = 0;
    uint8_t target_system = 1, target_component = 1;

    void encode(uint8_t *p) const
    {
      std::memset(p, 0, LEN);
      wrU16(p, 0, seq);
      wrU8(p, 2, target_system);
      wrU8(p, 3, target_component);
    }
  };

  // ID=73 MISSION_ITEM_INT，V2 载荷 38（V1 = 37），CRC_EXTRA=38
  struct MissionItemInt
  {
    static constexpr uint32_t ID = 73;
    static constexpr size_t LEN = 38;
    static constexpr uint8_t CRC_EXTRA = 38;

    float param1 = 0, param2 = 0, param3 = 0, param4 = 0;
    int32_t x = 0, y = 0;   // 纬度/经度 degE7
    float z = 0;            // 高度（frame 决定相对还是海拔）
    uint16_t seq = 0;
    uint16_t command = 0;
    uint8_t target_system = 1, target_component = 1;
    uint8_t frame = FRAME_GLOBAL_RELATIVE_ALT;
    uint8_t current = 0, autocontinue = 1;
    uint8_t mission_type = 0;

    // 便捷构造：航点(16)
    static MissionItemInt waypoint(uint16_t seq, double lat_deg, double lon_deg, double alt_rel_m,
                                   float hold_s = 0.f, float turn_radius = 0.f, float yaw_deg = 0.f)
    {
      MissionItemInt m;
      m.seq = seq; m.command = CMD_NAV_WAYPOINT;
      m.param1 = hold_s; m.param2 = turn_radius; m.param4 = yaw_deg;
      m.x = (int32_t)llround(lat_deg * 1e7);
      m.y = (int32_t)llround(lon_deg * 1e7);
      m.z = (float)alt_rel_m;
      m.frame = FRAME_GLOBAL_RELATIVE_ALT;
      m.current = (seq == 0) ? 1 : 0;
      return m;
    }
    // 便捷构造：起飞(22) —— 必须是任务第一条
    static MissionItemInt takeoff(uint16_t seq, double lat_deg, double lon_deg, double alt_rel_m)
    {
      MissionItemInt m;
      m.seq = seq; m.command = CMD_NAV_TAKEOFF;
      m.x = (int32_t)llround(lat_deg * 1e7);
      m.y = (int32_t)llround(lon_deg * 1e7);
      m.z = (float)alt_rel_m;
      m.frame = FRAME_GLOBAL_RELATIVE_ALT;
      m.current = (seq == 0) ? 1 : 0;
      return m;
    }
    static MissionItemInt speed(uint16_t seq, float ground_speed_mps)
    {
      MissionItemInt m;
      m.seq = seq; m.command = CMD_DO_SET_SPEED;
      m.param1 = 1.f; // 地速
      m.param2 = ground_speed_mps;
      m.current = 0;
      return m;
    }
    static MissionItemInt rtl(uint16_t seq)
    {
      MissionItemInt m;
      m.seq = seq; m.command = CMD_NAV_RETURN_TO_LAUNCH;
      m.current = 0;
      return m;
    }

    void encode(uint8_t *p) const
    {
      std::memset(p, 0, LEN);
      wrF32(p, 0, param1); wrF32(p, 4, param2); wrF32(p, 8, param3); wrF32(p, 12, param4);
      wrI32(p, 16, x); wrI32(p, 20, y); wrF32(p, 24, z);
      wrU16(p, 28, seq); wrU16(p, 30, command);
      wrU8(p, 32, target_system); wrU8(p, 33, target_component);
      wrU8(p, 34, frame); wrU8(p, 35, current); wrU8(p, 36, autocontinue);
      wrU8(p, 37, mission_type);
    }
    static bool decode(const uint8_t *p, size_t len, MissionItemInt &o)
    {
      if (len == 0) return false;
      uint8_t buf[LEN];
      padPayload(p, len, buf, LEN);
      p = buf;
      o.param1 = rdF32(p, 0); o.param2 = rdF32(p, 4); o.param3 = rdF32(p, 8); o.param4 = rdF32(p, 12);
      o.x = rdI32(p, 16); o.y = rdI32(p, 20); o.z = rdF32(p, 24);
      o.seq = rdU16(p, 28); o.command = rdU16(p, 30);
      o.target_system = rdU8(p, 32); o.target_component = rdU8(p, 33);
      o.frame = rdU8(p, 34); o.current = rdU8(p, 35); o.autocontinue = rdU8(p, 36);
      o.mission_type = rdU8(p, 37);
      return true;
    }
  };

} // namespace ego_mav

#endif
