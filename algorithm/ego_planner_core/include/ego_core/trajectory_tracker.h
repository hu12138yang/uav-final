#ifndef _TRAJECTORY_TRACKER_H_
#define _TRAJECTORY_TRACKER_H_

#include <Eigen/Eigen>
#include <memory>
#include <utility>
#include <vector>

#include <bspline_opt/uniform_bspline.h>
#include <ego_core/ego_local_planner.h> // BsplineTrajectory

namespace ego_planner
{

  /* status flags, mirroring quadrotor_msgs::PositionCommand */
  enum TrajectoryStatus : int
  {
    TRAJECTORY_STATUS_EMPTY = 0,
    TRAJECTORY_STATUS_READY = 1,
    TRAJECTORY_STATUS_COMPLETED = 3,
    TRAJECTORY_STATUS_ABORT = 4,
    TRAJECTORY_STATUS_ILLEGAL_START = 5,
    TRAJECTORY_STATUS_ILLEGAL_FINAL = 6,
    TRAJECTORY_STATUS_IMPOSSIBLE = 7
  };

  /* ROS-free equivalent of quadrotor_msgs::PositionCommand */
  struct QuadrotorCommand
  {
    double stamp = 0.0; // seconds
    Eigen::Vector3d position = Eigen::Vector3d::Zero();
    Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
    double yaw = 0.0;
    double yaw_dot = 0.0;
    double kx[3] = {0.0, 0.0, 0.0};
    double kv[3] = {0.0, 0.0, 0.0};
    unsigned int trajectory_id = 0;
    unsigned char trajectory_flag = TRAJECTORY_STATUS_EMPTY;
  };

  // Converts a planned B-spline trajectory into position/velocity/acceleration/
  // yaw flight-control commands, sampled at the current time.
  // The trajectory is referenced to BsplineTrajectory::start_time, so the
  // `now` argument must come from the same clock used when the trajectory was
  // planned.
  class TrajectoryTracker
  {
  public:
    TrajectoryTracker() {}
    ~TrajectoryTracker() {}

    // bind a newly planned trajectory
    void setTrajectory(const BsplineTrajectory &traj);

    // sample the trajectory and produce the command for time `now` (seconds).
    // returns false if there is no valid trajectory.
    bool getCommand(double now, QuadrotorCommand &cmd);

    void setTimeForward(double t) { time_forward_ = t; }
    bool hasTrajectory() const { return receive_traj_; }
    double duration() const { return traj_duration_; }
    double startTime() const { return start_time_; }
    unsigned int trajId() const { return traj_id_; }

  private:
    std::pair<double, double> calculateYaw(double t_cur, const Eigen::Vector3d &pos, double now, double time_last);

    std::vector<UniformBspline> traj_; // [position, velocity, acceleration]
    double traj_duration_ = 0.0;
    double start_time_ = 0.0;
    unsigned int traj_id_ = 0;
    bool receive_traj_ = false;

    // yaw control state
    double last_yaw_ = 0.0;
    double last_yaw_dot_ = 0.0;
    double time_forward_ = 1.0;
    double time_last_ = 0.0;
  };

} // namespace ego_planner

#endif
