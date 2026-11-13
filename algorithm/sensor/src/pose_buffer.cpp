#include "sensor/lidar_types.h"

#include <algorithm>
#include <cmath>

namespace ego_sensor
{

  void PoseBuffer::add(const TimedPose &pose)
  {
    if (!std::isfinite(pose.t) || !pose.p.allFinite() || !pose.q.coeffs().allFinite() ||
        std::abs(pose.q.norm() - 1.0) > 0.01) return;
    if (!buf_.empty() && pose.t < buf_.back().t)
      return; // 时间戳必须单调不减
    if (!buf_.empty() && pose.t == buf_.back().t) { buf_.back() = pose; return; }
    buf_.push_back(pose);
    while (buf_.size() > max_size_)
      buf_.pop_front();
  }

  namespace
  {
    bool positionAt(double t, const TimedPose &a, const TimedPose &b, TimedPose &out)
    {
      if (!a.position_anchor.valid && !b.position_anchor.valid) return true;
      const PositionAnchor *anchor = nullptr;
      for (const auto *candidate : {&a.position_anchor, &b.position_anchor}) {
        if (candidate->valid && candidate->t <= t + 1e-9 &&
            (!anchor || candidate->t > anchor->t)) anchor = candidate;
      }
      if (!anchor || !anchor->positionAt(t, out.p)) return false;
      out.t = t;
      out.position_anchor = *anchor;
      return true;
    }
  } // namespace

  bool PoseBuffer::poseAt(double t, TimedPose &out) const
  {
    if (buf_.empty())
      return false;

    // 轻微外插：点云的点可能比最新位姿晚一点点（多线程/传输延迟），
    // 允许在边界外 20 ms 内用端点值，避免整帧被判为不可用。
    const double kSlack = strict_ ? 0.0 : 0.02;
    if (!std::isfinite(t)) return false;
    if (t < buf_.front().t - kSlack || t > buf_.back().t + kSlack)
      return false;

    if (t <= buf_.front().t)
    {
      out = buf_.front();
      return positionAt(t, buf_.front(), buf_.front(), out);
    }
    if (t >= buf_.back().t)
    {
      out = buf_.back();
      return positionAt(t, buf_.back(), buf_.back(), out);
    }

    // 二分查找（缓冲可能上千个样本，每帧要查几千次，线性扫描扛不住）
    auto it = std::lower_bound(buf_.begin(), buf_.end(), t,
                               [](const TimedPose &a, double v) { return a.t < v; });
    if (it == buf_.begin())
    {
      out = buf_.front();
      return positionAt(t, buf_.front(), buf_.front(), out);
    }
    if (it == buf_.end())
    {
      out = buf_.back();
      return positionAt(t, buf_.back(), buf_.back(), out);
    }
    const TimedPose &b = *it;
    const TimedPose &a = *(it - 1);
    const double dt = b.t - a.t;
    if (dt > max_gap_ && t != b.t) return false;
    const double u = dt > 1e-9 ? (t - a.t) / dt : 1.0;
    out = TimedPose();
    out.t = t;
    if (!a.position_anchor.valid && !b.position_anchor.valid)
      out.p = a.p + (b.p - a.p) * u; // 其他位姿来源保留其位置插值语义
    out.q = a.q.slerp(u, b.q).normalized();
    return positionAt(t, a, b, out);
  }

} // namespace ego_sensor
