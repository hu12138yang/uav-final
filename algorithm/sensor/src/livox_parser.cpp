#include "sensor/livox_parser.h"

#include <cmath>
#include <cstring>

namespace ego_sensor
{

  namespace
  {
    // SDK 结构体里的 timestamp[8] 是小端 uint64（纳秒）
    inline uint64_t readTimestampNs(const uint8_t *b)
    {
      uint64_t v = 0;
      for (int i = 0; i < 8; ++i)
        v |= (uint64_t)b[i] << (8 * i);
      return v;
    }

    inline double range(const Eigen::Vector3d &p) { return p.norm(); }
  } // namespace

  uint64_t LivoxParser::packetTimeNs(const LivoxLidarEthernetPacket &pkt)
  {
    return readTimestampNs(pkt.timestamp);
  }

  double LivoxParser::packetTime(const LivoxLidarEthernetPacket &pkt, double t_epoch_offset)
  {
    return t_epoch_offset + (double)readTimestampNs(pkt.timestamp) * 1e-9;
  }

  bool LivoxParser::parsePacket(const LivoxLidarEthernetPacket &pkt, double t_epoch_offset,
                                LidarFrame &frame) const
  {
    const size_t header = offsetof(LivoxLidarEthernetPacket, data);
    size_t point_size = 0;
    switch (pkt.data_type) {
      case kLivoxLidarCartesianCoordinateHighData: point_size = sizeof(LivoxLidarCartesianHighRawPoint); break;
      case kLivoxLidarCartesianCoordinateLowData: point_size = sizeof(LivoxLidarCartesianLowRawPoint); break;
      case kLivoxLidarSphericalCoordinateData: point_size = sizeof(LivoxLidarSpherPoint); break;
      case kLivoxLidarDoubleEchoData: point_size = sizeof(LivoxLidarDoubleEchoRawPoint); break;
      default: return false;
    }
    if (pkt.dot_num == 0 || pkt.length < header ||
        pkt.dot_num > (pkt.length - header) / point_size || !std::isfinite(t_epoch_offset))
      return false;

    const double pkt_t = packetTime(pkt, t_epoch_offset);
    // time_interval 单位 0.1 us -> 1e-7 s
    // Protocol: time_interval is LAST minus FIRST, not the per-point interval.
    const double dt_point = pkt.dot_num > 1 ? (double)pkt.time_interval * 1e-7 / (pkt.dot_num - 1) : 0.0;

    // 逐帧累积：第一个包决定本帧基准时刻（对应 CustomMsg::timebase）
    if (frame.points.empty())
    {
      frame.t_base = pkt_t;
      frame.data_type = (int)pkt.data_type;
    }

    const uint8_t *raw = pkt.data;
    const size_t n = pkt.dot_num;

    auto push = [&](const Eigen::Vector3d &p, float refl, uint8_t tag, size_t idx, double t_abs) {
      const double r = range(p);
      if (!std::isfinite(r) || r < cfg_.min_range || r > cfg_.max_range)
        return; // 含全零点（无效回波）
      LidarPoint lp;
      lp.p = p;
      lp.reflectivity = refl;
      lp.tag = tag;
      lp.line = 0; // Mid 系列无激光线号（对应驱动里 dst_point->line = 0）
      lp.t_rel = t_abs - frame.t_base;
      frame.points.push_back(lp);
    };

    switch (pkt.data_type)
    {
    case kLivoxLidarCartesianCoordinateHighData: // int32, 单位 mm
    {
      const auto *pts = reinterpret_cast<const LivoxLidarCartesianHighRawPoint *>(raw);
      for (size_t i = 0; i < n; ++i)
        push(Eigen::Vector3d(pts[i].x * 1e-3, pts[i].y * 1e-3, pts[i].z * 1e-3), pts[i].reflectivity,
             pts[i].tag, i, pkt_t + i * dt_point);
      break;
    }
    case kLivoxLidarCartesianCoordinateLowData: // int16, 单位 cm
    {
      const auto *pts = reinterpret_cast<const LivoxLidarCartesianLowRawPoint *>(raw);
      for (size_t i = 0; i < n; ++i)
        push(Eigen::Vector3d(pts[i].x * 1e-2, pts[i].y * 1e-2, pts[i].z * 1e-2), pts[i].reflectivity,
             pts[i].tag, i, pkt_t + i * dt_point);
      break;
    }
    case kLivoxLidarSphericalCoordinateData: // depth(mm) + theta/phi(0.01 deg)
    {
      const auto *pts = reinterpret_cast<const LivoxLidarSpherPoint *>(raw);
      for (size_t i = 0; i < n; ++i)
      {
        const double d = pts[i].depth * 1e-3;
        const double th = pts[i].theta * 0.01 * M_PI / 180.0; // 极角（从 +Z 起）
        const double ph = pts[i].phi * 0.01 * M_PI / 180.0;   // 方位角
        push(Eigen::Vector3d(d * std::sin(th) * std::cos(ph), d * std::sin(th) * std::sin(ph),
                             d * std::cos(th)),
             pts[i].reflectivity, pts[i].tag, i, pkt_t + i * dt_point);
      }
      break;
    }
    case kLivoxLidarDoubleEchoData: // 双回波：x1/y1/z1 + ... + x2/y2/z2（同一脉冲）
    {
      const auto *pts = reinterpret_cast<const LivoxLidarDoubleEchoRawPoint *>(raw);
      for (size_t i = 0; i < n; ++i)
      {
        push(Eigen::Vector3d(pts[i].x1 * 1e-3, pts[i].y1 * 1e-3, pts[i].z1 * 1e-3), pts[i].reflectivity1,
             pts[i].tag1, i, pkt_t + i * dt_point);
        if (cfg_.keep_second_echo)
          push(Eigen::Vector3d(pts[i].x2 * 1e-3, pts[i].y2 * 1e-3, pts[i].z2 * 1e-3),
               pts[i].reflectivity2, pts[i].tag2, i, pkt_t + i * dt_point);
      }
      break;
    }
    default:
      return false;
    }

    return true;
  }

} // namespace ego_sensor
