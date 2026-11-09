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
    // 在 [a, b] 上按比例 u 插值
    TimedPose lerp(const TimedPose &a, const TimedPose &b, double u)
    {
      TimedPose out;
      out.t = a.t + (b.t - a.t) * u;
      out.p = a.p + (b.p - a.p) * u;
      out.q = a.q.slerp(u, b.q);
      out.q.normalize();
      return out;
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
      return true;
    }
    if (t >= buf_.back().t)
    {
      out = buf_.back();
      return true;
    }

    // 二分查找（缓冲可能上千个样本，每帧要查几千次，线性扫描扛不住）
    auto it = std::lower_bound(buf_.begin(), buf_.end(), t,
                               [](const TimedPose &a, double v) { return a.t < v; });
    if (it == buf_.begin())
    {
      out = buf_.front();
      return true;
    }
    if (it == buf_.end())
    {
      out = buf_.back();
      return true;
    }
    const TimedPose &b = *it;
    const TimedPose &a = *(it - 1);
    const double dt = b.t - a.t;
    if (dt > max_gap_ && t != b.t) return false;
    out = (dt > 1e-9) ? lerp(a, b, (t - a.t) / dt) : b;
    return true;
  }

} // namespace ego_sensor
