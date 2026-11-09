#ifndef _EGO_LOCAL_PLANNER_H_
#define _EGO_LOCAL_PLANNER_H_

#include <Eigen/Eigen>
#include <memory>
#include <vector>

#include <bspline_opt/bspline_optimizer.h>
#include <bspline_opt/uniform_bspline.h>
#include <plan_env/grid_map.h>
#include <traj_utils/plan_container.hpp>

namespace ego_planner
{

  /* ---------------- configuration ---------------- */

  struct EgoLocalPlannerConfig
  {
    BsplineOptimizerConfig opt; // optimization weights / dynamic limits
    // NOTE: opt.max_vel / opt.max_acc (used by the optimizer's feasibility cost)
    // should be consistent with max_vel / max_acc below.

    /* planner-manager parameters */
    double max_vel = 2.0;
    double max_acc = 6.0;
    double ctrl_pt_dist = 0.4;           // distance between adjacent B-spline control points
    double feasibility_tolerance = 0.05; // permitted ratio of vel/acc exceeding limits
    double planning_horizon = 7.5;       // used to detect an abnormally long initial path

    /* A* search pool */
    Eigen::Vector3i astar_pool_size = Eigen::Vector3i(100, 100, 100);
    double astar_step_size = 0.1;

    int bspline_order = 3;
  };

  /* ---------------- output trajectory ---------------- */

  // A planned local B-spline trajectory. It is the ROS-free
  // equivalent of traj_utils::Bspline and carries everything a tracker needs.
  struct BsplineTrajectory
  {
    int order = 3;
    int traj_id = 0;
    double start_time = 0.0; // seconds, given by the caller's clock
    Eigen::MatrixXd pos_pts; // 3 x N control points
    Eigen::VectorXd knots;   // knot vector
    UniformBspline bspline;  // built from pos_pts / order / interval
    double duration = 0.0;   // seconds

    bool empty() const { return pos_pts.cols() == 0; }
  };

  /* ---------------- planner ---------------- */

  // Mirrors the FSM's two planning entries:
  //   NEW_TRAJ    : initial path generated from a min-snap polynomial (FSM GEN_NEW_TRAJ)
  //   REPLAN_TRAJ : start point taken from the previous trajectory and the previous
  //                 trajectory reused as the initial path, with the FSM's fallback
  //                 chain (poly init -> random poly init). (FSM REPLAN_TRAJ)
  enum class PlanMode
  {
    NEW_TRAJ,
    REPLAN_TRAJ
  };

  class EgoLocalPlanner
  {
  public:
    EgoLocalPlanner() {}
    ~EgoLocalPlanner() {}

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // initialize internal optimizer / A* pool (call once)
    void init(const EgoLocalPlannerConfig &cfg);

    // bind the occupancy map (call after init, before plan)
    void setEnvironment(const GridMap::Ptr &map);

    // plan a collision-free local trajectory from the current state to a goal.
    // Returns true on success and fills `traj_out`.
    bool plan(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
              const Eigen::Vector3d &goal_pt, const Eigen::Vector3d &goal_vel, double time_now,
              BsplineTrajectory &traj_out, PlanMode mode = PlanMode::NEW_TRAJ);

    // convenience overload: goal velocity = 0, acceleration = 0
    bool plan(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel,
              const Eigen::Vector3d &goal_pt, double time_now, BsplineTrajectory &traj_out,
              PlanMode mode = PlanMode::NEW_TRAJ);

    int continousFailuresCount() const { return continous_failures_count_; }
    const LocalTrajData &localTraj() const { return local_data_; }

  private:
    EgoLocalPlannerConfig cfg_;

    GridMap::Ptr grid_map_;
    BsplineOptimizer::Ptr bspline_optimizer_;

    LocalTrajData local_data_;

    int continous_failures_count_{0};
    bool flag_first_call_{true};
    bool flag_force_polynomial_{false};

    bool reboundReplan(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                       const Eigen::Vector3d &local_target_pt, const Eigen::Vector3d &local_target_vel,
                       bool flag_polyInit, bool flag_randomPolyTraj, double time_now);

    bool refineTrajAlgo(UniformBspline &traj, std::vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts,
                        Eigen::MatrixXd &optimal_control_points);

    void reparamBspline(UniformBspline &bspline, std::vector<Eigen::Vector3d> &start_end_derivative, double ratio,
                        Eigen::MatrixXd &ctrl_pts, double &dt, double &time_inc);

    void updateTrajInfo(const UniformBspline &position_traj, double time_now);
  };

} // namespace ego_planner

#endif
