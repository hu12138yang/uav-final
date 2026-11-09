#ifndef _SENSOR_LIDAR_TYPES_H_
#define _SENSOR_LIDAR_TYPES_H_

#include <Eigen/Eigen>

#include <cstddef>
#include <deque>
#include <vector>

namespace ego_sensor
{

  // ---------------------------------------------------------------------------
  // 感知输入的边界（这就是本模块存在的意义）
  //
  // 上游只负责给出两样东西：
  //   1. 带时间戳的位姿   —— 实机上由飞控（姿态）+ GPS/RTK（位置）融合得到
  //   2. 一帧原始激光点云 —— 实机上由 Livox-SDK2 的回调给出（尚未转世界系）
  // 本模块把它们变成规划器需要的"世界系点云"，并把最容易出错的一步
  // （按每点自己的采样时刻做去畸变）固定在代码里。
  // ---------------------------------------------------------------------------

  // 一帧位姿采样，局部 ENU / 世界系。规划器不关心它是 VIO、RTK 还是融合出来的。
  struct TimedPose
  {
    double t = 0.0;                                        // 秒，与 LidarPoint::t_rel / LidarFrame::t_base 同一时钟
    Eigen::Vector3d p = Eigen::Vector3d::Zero();            // 世界系位置
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();  // 机体系 -> 世界系
  };

  // 时间戳位姿缓冲 + 插值。
  // 用途：一个 10 Hz 的激光帧持续 100 ms，期间飞机已经移动/转动，所以每个点
  // 必须用"它自己的采样时刻"的位姿做变换，而不是整帧共用一个位姿。
  class PoseBuffer
  {
  public:
    // 追加一个采样（要求时间单调不减，否则丢弃）
    void add(const TimedPose &pose);
    // 取 t 时刻的位姿（线性 + slerp 插值）；覆盖不到返回 false
    bool poseAt(double t, TimedPose &out) const;

    bool empty() const { return buf_.empty(); }
    double oldestTime() const { return buf_.empty() ? 0.0 : buf_.front().t; }
    double newestTime() const { return buf_.empty() ? 0.0 : buf_.back().t; }
    size_t size() const { return buf_.size(); }
    void setMaxSize(size_t n) { max_size_ = n; }
    void setStrictCoverage(bool strict) { strict_ = strict; }
    void setMaxGap(double seconds) { max_gap_ = seconds; }

  private:
    std::deque<TimedPose> buf_;
    // 只保留最近一小段：查询永远是"当前时刻附近"，长期历史没用。
    // 10 Hz 激光一帧 100 ms，500 个样本（100 Hz 下 5 s）足够富余。
    size_t max_size_ = 500;
    bool strict_ = false;
    double max_gap_ = 0.1;
  };

  // 一个激光回波，坐标在**传感器坐标系**下（米），带自己的采样时刻。
  // 字段与 livox_ros_driver 的 CustomPoint 对齐（offset_time / xyz / reflectivity / tag / line）。
  struct LidarPoint
  {
    Eigen::Vector3d p = Eigen::Vector3d::Zero(); // 传感器系，米
    double t_rel = 0.0;                          // 相对本帧基准时刻（秒）—— 对应 CustomPoint::offset_time
    float reflectivity = 0.0f;                   // 0~255
    uint8_t tag = 0;                             // Livox tag
    uint8_t line = 0;                            // 激光线号（Mid 系列无，固定 0）
  };

  // 一帧原始点云：未去畸变、未转世界系。
  // 对应 livox_ros_driver 的 CustomMsg：timebase + points[offset_time]。
  struct LidarFrame
  {
    double t_base = 0.0; // 本帧基准时刻 = 第一个包的雷达时间（位姿时钟）。
                         // 由驱动做时间同步后给出，不在这里猜 —— 见文档"时间同步"。
    bool payload_crc_present = true; // false when any packet omits its payload CRC
    uint8_t lidar_id = 0; // 多雷达时的设备号
    int data_type = 0;    // LivoxLidarPointDataType，仅用于排查
    uint8_t time_type = 0; // 0=unsynced, 1=PTP, 2=GPS
    std::vector<LidarPoint> points;
  };

  // 外参：传感器 -> 机体系
  struct Extrinsic
  {
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t = Eigen::Vector3d::Zero();
  };

} // namespace ego_sensor

#endif
