#ifndef _SENSOR_SCAN_ASSEMBLER_H_
#define _SENSOR_SCAN_ASSEMBLER_H_

#include <vector>

#include "sensor/lidar_types.h"

namespace ego_sensor
{

  // 把一帧原始点云（传感器系）变成世界系点云。
  // 关键点：**逐点去畸变** —— 每个点用它自己采样时刻的位姿，
  // 而不是整帧共用一个位姿。
  class ScanAssembler
  {
  public:
    void setExtrinsic(const Extrinsic &T_body_sensor) { T_body_sensor_ = T_body_sensor; }
    const Extrinsic &extrinsic() const { return T_body_sensor_; }

    // 逐点去畸变。返回 false = 位姿缓冲覆盖不了该帧 → 调用方应丢弃这一帧，
    // 不要用错位姿去建图（insert-only 地图会把错误永久留下）。
    bool toWorld(const LidarFrame &frame, const PoseBuffer &poses,
                 std::vector<Eigen::Vector3d> &out) const;

    // 对照组：整帧共用 frame.t_base 一个位姿。运动时会"糊"，
    // 只用于量化去畸变到底值多少。
    bool toWorldNoDeskew(const LidarFrame &frame, const PoseBuffer &poses,
                         std::vector<Eigen::Vector3d> &out) const;

  private:
    Extrinsic T_body_sensor_;
  };

} // namespace ego_sensor

#endif
