#ifndef _ONBOARD_GPS_DEAD_RECKONING_H_
#define _ONBOARD_GPS_DEAD_RECKONING_H_

// GPS 丢星后的短时航位推算（dead reckoning），输出经纬高。
// 只对速度做一次积分，不对加速度做二次积分。
//   位置 = 最后一帧有效 GPS 位置 + ∫ v·dt
// 速度取自 GLOBAL_POSITION_INT 的 vx/vy/vz（飞控 EKF 输出，本身已经融合过 IMU）。
// ⚠️ 这个估计只用于「上报地面站」和「航点进度判断」，**不参与控制**：
//    协议里没有外部定位输入（VISION_POSITION_ESTIMATE / ODOMETRY），
//    飞控不接受我们算的位置，控制仍然走飞控自己的 EKF。

#include <algorithm>
#include <cmath>

#include <Eigen/Eigen>

namespace ego_onboard
{

  struct GeoFix
  {
    double lat_deg = 0.0, lon_deg = 0.0, alt_m = 0.0;
    bool valid = false;
  };

  class GpsDeadReckoner
  {
  public:
    void reset()
    {
      anchor_ = GeoFix();
      offset_.setZero();
      v_.setZero();
      t_last_ = 0.0;
      t_anchor_ = 0.0;
      have_ = false;
    }

    // 定位有效：把锚点钉到这一点，位移清零
    void onValidFix(const GeoFix &fix, double now)
    {
      anchor_ = fix;
      anchor_.valid = true;
      offset_.setZero();
      v_.setZero();
      t_last_ = now;
      t_anchor_ = now;
      have_ = true;
    }

    // 位置流每来一帧喂一次速度（ENU，m/s）；梯形积分
    void onVelocity(const Eigen::Vector3d &vel_enu, double now)
    {
      if (!have_)
      {
        t_last_ = now;
        v_ = vel_enu;
        return;
      }
      const double dt = std::max(0.0, std::min(0.2, now - t_last_)); // 限幅，防时间跳变
      offset_ += 0.5 * (v_ + vel_enu) * dt;
      v_ = vel_enu;
      t_last_ = now;
    }

    // 推算的经纬高；age_s = 距最后一次有效定位多久
    bool estimate(double now, GeoFix &out, double &age_s) const
    {
      if (!have_)
        return false;
      constexpr double kR = 6378137.0;
      constexpr double kDeg = 57.29577951308232; // 180/pi
      const double lat0r = anchor_.lat_deg / kDeg;
      out.lat_deg = anchor_.lat_deg + offset_.y() / kR * kDeg;            // +y = 北
      out.lon_deg = anchor_.lon_deg + offset_.x() / (kR * std::cos(lat0r)) * kDeg; // +x = 东
      out.alt_m = anchor_.alt_m + offset_.z();
      out.valid = true;
      age_s = now - t_anchor_;
      return true;
    }

    bool hasAnchor() const { return have_; }
    const Eigen::Vector3d &offset() const { return offset_; }

  private:
    GeoFix anchor_;
    Eigen::Vector3d offset_ = Eigen::Vector3d::Zero(); // 自锚点起的 ENU 位移
    Eigen::Vector3d v_ = Eigen::Vector3d::Zero();      // 上一次速度（梯形积分用）
    double t_last_ = 0.0;
    double t_anchor_ = 0.0;
    bool have_ = false;
  };

} // namespace ego_onboard

#endif
