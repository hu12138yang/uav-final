#include "ego_core/trajectory_tracker.h"

#include <algorithm>
#include <cmath>
#include <iostream>

namespace ego_planner
{

  void TrajectoryTracker::setTrajectory(const BsplineTrajectory &traj)
  {
    if (traj.empty())
      return;

    UniformBspline pos_traj(traj.pos_pts, traj.order, 0.1);
    if (traj.knots.size() > 0)
      pos_traj.setKnot(traj.knots);

    start_time_ = traj.start_time;
    traj_id_ = traj.traj_id;

    traj_.clear();
    traj_.push_back(pos_traj);
    traj_.push_back(traj_[0].getDerivative());
    traj_.push_back(traj_[1].getDerivative());

    traj_duration_ = traj_[0].getTimeSum();

    receive_traj_ = true;
    time_last_ = start_time_;
  }

  bool TrajectoryTracker::getCommand(double now, QuadrotorCommand &cmd)
  {
    if (!receive_traj_ || traj_.size() < 3)
      return false;

    double t_cur = now - start_time_;
    if (t_cur < 0.0)
      t_cur = 0.0;

    Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero());
    std::pair<double, double> yaw_yawdot(0.0, 0.0);

    if (t_cur < traj_duration_)
    {
      pos = traj_[0].evaluateDeBoorT(t_cur);
      vel = traj_[1].evaluateDeBoorT(t_cur);
      acc = traj_[2].evaluateDeBoorT(t_cur);

      /*** calculate yaw ***/
      yaw_yawdot = calculateYaw(t_cur, pos, now, time_last_);
    }
    else // t_cur >= traj_duration_: hover at the end of the trajectory
    {
      pos = traj_[0].evaluateDeBoorT(traj_duration_);
      vel.setZero();
      acc.setZero();

      yaw_yawdot.first = last_yaw_;
      yaw_yawdot.second = 0.0;
    }
    time_last_ = now;

    cmd.stamp = now;
    cmd.trajectory_flag = TRAJECTORY_STATUS_READY;
    cmd.trajectory_id = traj_id_;

    cmd.position = pos;
    cmd.velocity = vel;
    cmd.acceleration = acc;

    cmd.yaw = yaw_yawdot.first;
    cmd.yaw_dot = yaw_yawdot.second;

    last_yaw_ = cmd.yaw;

    return true;
  }

  std::pair<double, double> TrajectoryTracker::calculateYaw(double t_cur, const Eigen::Vector3d &pos, double now, double time_last)
  {
    constexpr double PI = 3.1415926;
    constexpr double YAW_DOT_MAX_PER_SEC = PI;

    std::pair<double, double> yaw_yawdot(0.0, 0.0);
    double yaw = 0.0;
    double yawdot = 0.0;

    double dt = now - time_last;
    if (dt <= 1e-6)
      dt = 1e-6;

    Eigen::Vector3d dir = t_cur + time_forward_ <= traj_duration_ ? traj_[0].evaluateDeBoorT(t_cur + time_forward_) - pos : traj_[0].evaluateDeBoorT(traj_duration_) - pos;
    double yaw_temp = dir.norm() > 0.1 ? atan2(dir(1), dir(0)) : last_yaw_;
    double max_yaw_change = YAW_DOT_MAX_PER_SEC * dt;
    if (yaw_temp - last_yaw_ > PI)
    {
      if (yaw_temp - last_yaw_ - 2 * PI < -max_yaw_change)
      {
        yaw = last_yaw_ - max_yaw_change;
        if (yaw < -PI)
          yaw += 2 * PI;

        yawdot = -YAW_DOT_MAX_PER_SEC;
      }
      else
      {
        yaw = yaw_temp;
        if (yaw - last_yaw_ > PI)
          yawdot = -YAW_DOT_MAX_PER_SEC;
        else
          yawdot = (yaw_temp - last_yaw_) / dt;
      }
    }
    else if (yaw_temp - last_yaw_ < -PI)
    {
      if (yaw_temp - last_yaw_ + 2 * PI > max_yaw_change)
      {
        yaw = last_yaw_ + max_yaw_change;
        if (yaw > PI)
          yaw -= 2 * PI;

        yawdot = YAW_DOT_MAX_PER_SEC;
      }
      else
      {
        yaw = yaw_temp;
        if (yaw - last_yaw_ < -PI)
          yawdot = YAW_DOT_MAX_PER_SEC;
        else
          yawdot = (yaw_temp - last_yaw_) / dt;
      }
    }
    else
    {
      if (yaw_temp - last_yaw_ < -max_yaw_change)
      {
        yaw = last_yaw_ - max_yaw_change;
        if (yaw < -PI)
          yaw += 2 * PI;

        yawdot = -YAW_DOT_MAX_PER_SEC;
      }
      else if (yaw_temp - last_yaw_ > max_yaw_change)
      {
        yaw = last_yaw_ + max_yaw_change;
        if (yaw > PI)
          yaw -= 2 * PI;

        yawdot = YAW_DOT_MAX_PER_SEC;
      }
      else
      {
        yaw = yaw_temp;
        if (yaw - last_yaw_ > PI)
          yawdot = -YAW_DOT_MAX_PER_SEC;
        else if (yaw - last_yaw_ < -PI)
          yawdot = YAW_DOT_MAX_PER_SEC;
        else
          yawdot = (yaw_temp - last_yaw_) / dt;
      }
    }

    if (fabs(yaw - last_yaw_) <= max_yaw_change)
      yaw = 0.5 * last_yaw_ + 0.5 * yaw; // nieve LPF
    yawdot = 0.5 * last_yaw_dot_ + 0.5 * yawdot;
    last_yaw_ = yaw;
    last_yaw_dot_ = yawdot;

    yaw_yawdot.first = yaw;
    yaw_yawdot.second = yawdot;

    return yaw_yawdot;
  }

} // namespace ego_planner
