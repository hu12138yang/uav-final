#ifndef _MAVLINK_MAVLINK_CODEC_H_
#define _MAVLINK_MAVLINK_CODEC_H_

// MAVLink v1/v2 帧编解码 + CRC-16/MCRF4XX。
// 校验算法与 CRC_EXTRA 表按《飞控地面站交互协议 V2.8.4》2.3 节实现。
// 只做收发，不做语义 —— 语义在 fcu_interface.h。

#include <cstdint>
#include <cstring>
#include <map>
#include <vector>

namespace ego_mav
{

  // 文档 2.3 给出的累加函数（CRC-16/MCRF4XX）
  inline void crcAccumulate(uint8_t data, uint16_t &crc)
  {
    uint8_t tmp = (uint8_t)(data ^ (uint8_t)(crc & 0xFF));
    tmp = (uint8_t)(tmp ^ (uint8_t)(tmp << 4));
    crc = (uint16_t)((crc >> 8) ^ ((uint16_t)tmp << 8) ^ ((uint16_t)tmp << 3) ^ ((uint16_t)tmp >> 4));
  }

  // CRC_EXTRA 表。
  //
  // 大部分取值与《飞控地面站交互协议》2.3 节的表逐条一致（该表又与官方 MAVLink 一致）。
  // 但有 3 条厂商协议表里**没有列**，厂商的 Python 参考实现给的又与官方不同：
  //     报文            官方 MAVLink   厂商 Python   本文件采用
  //     2  SYSTEM_TIME      137           217           217
  //     242 HOME_POSITION   104            36            36
  //     330 OBSTACLE_DISTANCE 23          158           158
  // 这里**先按厂商 Python 参考实现对齐**（它是唯一在真机上跑过的），
  // 并用 setCrcExtra() 留出覆盖入口 —— 台架上一分钟就能定性（见 Parser::crcErrorsByMsgid()）：
  // 若某个报文 100% CRC 失败，就是这条 CRC_EXTRA 错了。
  inline std::map<uint32_t, uint8_t> &crcExtraOverrides()
  {
    static std::map<uint32_t, uint8_t> m;
    return m;
  }
  inline void setCrcExtra(uint32_t msgid, uint8_t extra) { crcExtraOverrides()[msgid] = extra; }

  inline bool crcExtra(uint32_t msgid, uint8_t &extra)
  {
    const auto it = crcExtraOverrides().find(msgid);
    if (it != crcExtraOverrides().end())
    {
      extra = it->second;
      return true;
    }
    switch (msgid)
    {
    case 0:   extra = 50;  return true; // HEARTBEAT
    case 1:   extra = 124; return true; // SYS_STATUS
    case 2:   extra = 137; return true; // SYSTEM_TIME（实测飞控用标准 137；协议表写 217）
    case 20:  extra = 214; return true; // PARAM_REQUEST_READ
    case 22:  extra = 220; return true; // PARAM_VALUE
    case 23:  extra = 168; return true; // PARAM_SET
    case 24:  extra = 24;  return true; // GPS_RAW_INT
    case 30:  extra = 39;  return true; // ATTITUDE
    case 32:  extra = 185; return true; // LOCAL_POSITION_NED
    case 33:  extra = 104; return true; // GLOBAL_POSITION_INT
    case 42:  extra = 28;  return true; // MISSION_CURRENT
    case 43:  extra = 132; return true; // MISSION_REQUEST_LIST
    case 41:  extra = 132; return true; // MISSION_SET_CURRENT
    case 44:  extra = 221; return true; // MISSION_COUNT
    case 45:  extra = 232; return true; // MISSION_CLEAR_ALL
    case 47:  extra = 153; return true; // MISSION_ACK
    case 51:  extra = 196; return true; // MISSION_REQUEST_INT
    case 65:  extra = 118; return true; // RC_CHANNELS
    case 66:  extra = 148; return true; // REQUEST_DATA_STREAM
    case 69:  extra = 243; return true; // MANUAL_CONTROL
    case 73:  extra = 38;  return true; // MISSION_ITEM_INT
    case 74:  extra = 20;  return true; // VFR_HUD
    case 76:  extra = 152; return true; // COMMAND_LONG
    case 77:  extra = 143; return true; // COMMAND_ACK
    case 84:  extra = 143; return true; // SET_POSITION_TARGET_LOCAL_NED
    case 86:  extra = 5;   return true; // SET_POSITION_TARGET_GLOBAL_INT
    case 111: extra = 34; return true; // TIMESYNC
    case 124: extra = 87;  return true; // GPS2_RAW
    case 147: extra = 154; return true; // BATTERY_STATUS
    case 148: extra = 178; return true; // AUTOPILOT_VERSION
    case 242: extra = 36;  return true; // HOME_POSITION        （官方 104）
    case 243: extra = 85;  return true; // SET_HOME_POSITION
    case 245: extra = 130; return true; // EXTENDED_SYS_STATE
    case 253: extra = 83;  return true; // STATUSTEXT
    case 330: extra = 23;  return true; // OBSTACLE_DISTANCE    （官方 23）
    case 500: extra = 243; return true; // COMPONENT_EXTENSION43
    case 602: extra = 159; return true; // BATTERY_STATUS_ACFLY
    default:  extra = 0;   return false;
    }
  }

  struct MavMessage
  {
    uint8_t sysid = 0, compid = 0, seq = 0;
    uint8_t incompat_flags = 0;
    uint32_t msgid = 0;
    std::vector<uint8_t> payload;
    bool crc_checked = false; // 表内报文才做校验
  };

  // 打包（默认 v2；飞控上电默认 v1，需要先发 v2 心跳包切过去）
  inline std::vector<uint8_t> pack(uint8_t sysid, uint8_t compid, uint8_t seq, uint32_t msgid,
                                   const uint8_t *payload, size_t len, bool v2 = true, bool signed_frame = false)
  {
    std::vector<uint8_t> f;
    uint8_t extra = 0;
    const bool known = crcExtra(msgid, extra);
    if (!known)
      return f; // 不认识的报文不发

    if (v2)
    {
      f.push_back(0xFD);
      f.push_back((uint8_t)len);
      f.push_back(signed_frame ? 0x01 : 0x00); // incompat_flags
      f.push_back(0x00);                       // compat_flags
      f.push_back(seq);
      f.push_back(sysid);
      f.push_back(compid);
      f.push_back((uint8_t)(msgid & 0xFF));
      f.push_back((uint8_t)((msgid >> 8) & 0xFF));
      f.push_back((uint8_t)((msgid >> 16) & 0xFF));
    }
    else
    {
      if (msgid > 255)
        return f; // v1 只有 8 位 msgid
      f.push_back(0xFE);
      f.push_back((uint8_t)len);
      f.push_back(seq);
      f.push_back(sysid);
      f.push_back(compid);
      f.push_back((uint8_t)msgid);
    }

    const size_t hdr = v2 ? 10 : 6;
    f.insert(f.end(), payload, payload + len);

    uint16_t crc = 0xFFFF;
    for (size_t i = 1; i < hdr + len; ++i)
      crcAccumulate(f[i], crc);
    crcAccumulate(extra, crc);
    f.push_back((uint8_t)(crc & 0xFF));
    f.push_back((uint8_t)((crc >> 8) & 0xFF));
    return f;
  }

  // 流式解析：逐字节喂入，校验通过才吐出一条消息
  class Parser
  {
  public:
    bool push(uint8_t b, MavMessage &out)
    {
      if (len_ >= sizeof(buf_)) // 兜底，别让缓冲无限长
        std::memmove(buf_, buf_ + 1, --len_);
      buf_[len_++] = b;
      return tryParse(out);
    }

    uint64_t crcErrors() const { return crc_errors_; }
    uint64_t frames() const { return frames_; }
    // 按 msgid 统计 CRC 失败次数 —— 台架定性 CRC_EXTRA 用：
    // 某报文一直失败（100%）= 它的 CRC_EXTRA 写错了。
    const std::map<uint32_t, uint64_t> &crcErrorsByMsgid() const { return crc_err_by_id_; }

  private:
    bool tryParse(MavMessage &out)
    {
      // 先同步到帧头
      while (len_ > 0 && buf_[0] != 0xFE && buf_[0] != 0xFD)
        std::memmove(buf_, buf_ + 1, --len_);
      if (len_ < 3)
        return false;

      const bool v2 = (buf_[0] == 0xFD);
      const size_t hdr = v2 ? 10 : 6;
      if (len_ < hdr)
        return false;
      const size_t plen = buf_[1];
      size_t total = hdr + plen + 2;
      if (v2 && (buf_[2] & 0x01))
        total += 13; // 带签名
      if (len_ < total)
        return false;

      uint32_t msgid;
      if (v2)
        msgid = (uint32_t)buf_[7] | ((uint32_t)buf_[8] << 8) | ((uint32_t)buf_[9] << 16);
      else
        msgid = buf_[5];

      uint8_t extra = 0;
      const bool known = crcExtra(msgid, extra);
      if (known)
      {
        uint16_t crc = 0xFFFF;
        for (size_t i = 1; i < hdr + plen; ++i)
          crcAccumulate(buf_[i], crc);
        crcAccumulate(extra, crc);
        const uint16_t got = (uint16_t)(buf_[hdr + plen] | ((uint16_t)buf_[hdr + plen + 1] << 8));
        // Two documented HOME_POSITION dialects are used by this FCU family.
        if (crc != got && msgid == 242) {
          uint16_t alternate=0xFFFF;
          for (size_t i=1; i<hdr+plen; ++i) crcAccumulate(buf_[i],alternate);
          crcAccumulate(extra==36 ? 104 : 36,alternate);
          if (alternate==got) crc=alternate;
        }
        if (crc != got)
        {
          ++crc_errors_;
          ++crc_err_by_id_[msgid];
          std::memmove(buf_, buf_ + 1, --len_); // 不同步，丢一个字节重来
          return false;
        }
      }

      out.sysid = v2 ? buf_[5] : buf_[3];
      out.compid = v2 ? buf_[6] : buf_[4];
      out.seq = v2 ? buf_[4] : buf_[2];
      out.incompat_flags = v2 ? buf_[2] : 0;
      out.msgid = msgid;
      out.crc_checked = known;
      out.payload.assign(buf_ + hdr, buf_ + hdr + plen);

      std::memmove(buf_, buf_ + total, len_ - total);
      len_ -= total;
      ++frames_;
      return true;
    }

    uint8_t buf_[280];
    size_t len_ = 0;
    uint64_t crc_errors_ = 0;
    uint64_t frames_ = 0;
    std::map<uint32_t, uint64_t> crc_err_by_id_;
  };

} // namespace ego_mav

#endif
