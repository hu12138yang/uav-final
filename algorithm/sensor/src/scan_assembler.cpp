#include "sensor/scan_assembler.h"

namespace ego_sensor
{

  bool ScanAssembler::toWorld(const LidarFrame &frame, const PoseBuffer &poses,
                              std::vector<Eigen::Vector3d> &out) const
  {
    out.clear();
    if (frame.points.empty() || !std::isfinite(frame.t_base)) return false;
    out.reserve(frame.points.size());

    for (const auto &lp : frame.points)
    {
      if (!lp.p.allFinite() || !std::isfinite(lp.t_rel) || lp.t_rel < 0) { out.clear(); return false; }
      TimedPose pose;
      // 用这个点**自己的采样时刻**的位姿
      if (!poses.poseAt(frame.t_base + lp.t_rel, pose))
      { out.clear(); return false; }

      const Eigen::Vector3d p_body = T_body_sensor_.R * lp.p + T_body_sensor_.t;
      out.push_back(pose.p + pose.q * p_body);
    }
    return true;
  }

  bool ScanAssembler::toWorldNoDeskew(const LidarFrame &frame, const PoseBuffer &poses,
                                      std::vector<Eigen::Vector3d> &out) const
  {
    out.clear();
    out.reserve(frame.points.size());

    TimedPose pose;
    if (!poses.poseAt(frame.t_base, pose))
      return false;

    for (const auto &lp : frame.points)
    {
      const Eigen::Vector3d p_body = T_body_sensor_.R * lp.p + T_body_sensor_.t;
      out.push_back(pose.p + pose.q * p_body);
    }
    return true;
  }

} // namespace ego_sensor
