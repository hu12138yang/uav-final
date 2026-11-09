#include "ego_core/ego_local_planner.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>

namespace ego_planner
{

  void EgoLocalPlanner::init(const EgoLocalPlannerConfig &cfg)
  {
    cfg_ = cfg;
    continous_failures_count_ = 0;
    flag_first_call_ = true;
    flag_force_polynomial_ = false;
    local_data_.traj_id_ = 0;

    bspline_optimizer_.reset(new BsplineOptimizer);
    bspline_optimizer_->setParam(cfg_.opt);
    bspline_optimizer_->a_star_.reset(new AStar);

    if (grid_map_)
    {
      bspline_optimizer_->setEnvironment(grid_map_);
      bspline_optimizer_->a_star_->initGridMap(grid_map_, cfg_.astar_pool_size);
    }
  }

  void EgoLocalPlanner::setEnvironment(const GridMap::Ptr &map)
  {
    grid_map_ = map;
    if (bspline_optimizer_)
    {
      bspline_optimizer_->setEnvironment(grid_map_);
      bspline_optimizer_->a_star_->initGridMap(grid_map_, cfg_.astar_pool_size);
    }
  }

  bool EgoLocalPlanner::plan(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                             const Eigen::Vector3d &goal_pt, const Eigen::Vector3d &goal_vel, double time_now,
                             BsplineTrajectory &traj_out, PlanMode mode)
  {
    if (!bspline_optimizer_ || !grid_map_)
    {
      std::cerr << "[EgoLocalPlanner] not initialized (call init() and setEnvironment() first)." << std::endl;
      return false;
    }

    const int TRIAL_TIMES = 10;
    bool ok = false;

    if (mode == PlanMode::REPLAN_TRAJ && local_data_.duration_ > 0.0)
    {
      /* FSM planFromCurrentTraj: start from the current point of the previous
         trajectory, and first try to reuse that trajectory as the initial path. */
      double t_cur = time_now - local_data_.start_time_;
      if (t_cur < 0)
        t_cur = 0;
      if (t_cur > local_data_.duration_)
        t_cur = local_data_.duration_;

      Eigen::Vector3d s_p = local_data_.position_traj_.evaluateDeBoorT(t_cur);
      Eigen::Vector3d s_v = local_data_.velocity_traj_.evaluateDeBoorT(t_cur);
      Eigen::Vector3d s_a = local_data_.acceleration_traj_.evaluateDeBoorT(t_cur);

      ok = reboundReplan(s_p, s_v, s_a, goal_pt, goal_vel, false, false, time_now);
      if (!ok)
        ok = reboundReplan(s_p, s_v, s_a, goal_pt, goal_vel, true, false, time_now);
      if (!ok)
        for (int i = 0; i < TRIAL_TIMES; i++)
        {
          ok = reboundReplan(s_p, s_v, s_a, goal_pt, goal_vel, true, true, time_now);
          if (ok)
            break;
        }
    }
    else
    {
      /* FSM GEN_NEW_TRAJ: initial path generated from a min-snap polynomial. */
      ok = reboundReplan(start_pt, start_vel, start_acc, goal_pt, goal_vel, true, false, time_now);
      if (!ok)
        for (int i = 0; i < TRIAL_TIMES; i++)
        {
          ok = reboundReplan(start_pt, start_vel, start_acc, goal_pt, goal_vel, true, true, time_now);
          if (ok)
            break;
        }
    }

    if (!ok)
      return false;

    traj_out.order = cfg_.bspline_order;
    traj_out.traj_id = local_data_.traj_id_;
    traj_out.start_time = local_data_.start_time_;
    traj_out.bspline = local_data_.position_traj_;
    traj_out.pos_pts = local_data_.position_traj_.getControlPoint();
    traj_out.knots = local_data_.position_traj_.getKnot();
    traj_out.duration = local_data_.duration_;
    return true;
  }

  bool EgoLocalPlanner::plan(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel,
                             const Eigen::Vector3d &goal_pt, double time_now, BsplineTrajectory &traj_out, PlanMode mode)
  {
    return plan(start_pt, start_vel, Eigen::Vector3d::Zero(), goal_pt, Eigen::Vector3d::Zero(), time_now, traj_out, mode);
  }

  bool EgoLocalPlanner::reboundReplan(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                      const Eigen::Vector3d &local_target_pt, const Eigen::Vector3d &local_target_vel,
                                      bool flag_polyInit, bool flag_randomPolyTraj, double time_now)
  {
    if ((start_pt - local_target_pt).norm() < 0.2)
    {
      std::cout << "[EgoLocalPlanner] Close to goal." << std::endl;
      continous_failures_count_++;
      return false;
    }

    bspline_optimizer_->setLocalTargetPt(local_target_pt);

    auto t_start = std::chrono::steady_clock::now();
    std::chrono::duration<double> t_init, t_opt, t_refine;

    /*** STEP 1: INIT ***/
    double ts = (start_pt - local_target_pt).norm() > 0.1 ? cfg_.ctrl_pt_dist / cfg_.max_vel * 1.5 : cfg_.ctrl_pt_dist / cfg_.max_vel * 5;
    vector<Eigen::Vector3d> point_set, start_end_derivatives;
    bool flag_regenerate = false;
    do
    {
      point_set.clear();
      start_end_derivatives.clear();
      flag_regenerate = false;

      if (flag_first_call_ || flag_polyInit || flag_force_polynomial_) // Initial path generated from a min-snap traj by order.
      {
        flag_first_call_ = false;
        flag_force_polynomial_ = false;

        PolynomialTraj gl_traj;

        double dist = (start_pt - local_target_pt).norm();
        double time = pow(cfg_.max_vel, 2) / cfg_.max_acc > dist ? sqrt(dist / cfg_.max_acc) : (dist - pow(cfg_.max_vel, 2) / cfg_.max_acc) / cfg_.max_vel + 2 * cfg_.max_vel / cfg_.max_acc;

        if (!flag_randomPolyTraj)
        {
          gl_traj = PolynomialTraj::one_segment_traj_gen(start_pt, start_vel, start_acc, local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), time);
        }
        else
        {
          Eigen::Vector3d horizen_dir = ((start_pt - local_target_pt).cross(Eigen::Vector3d(0, 0, 1))).normalized();
          Eigen::Vector3d vertical_dir = ((start_pt - local_target_pt).cross(horizen_dir)).normalized();
          Eigen::Vector3d random_inserted_pt = (start_pt + local_target_pt) / 2 +
                                               (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * horizen_dir * 0.8 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989) +
                                               (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * vertical_dir * 0.4 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989);
          Eigen::MatrixXd pos(3, 3);
          pos.col(0) = start_pt;
          pos.col(1) = random_inserted_pt;
          pos.col(2) = local_target_pt;
          Eigen::VectorXd t(2);
          t(0) = t(1) = time / 2;
          gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, local_target_vel, start_acc, Eigen::Vector3d::Zero(), t);
        }

        double t;
        bool flag_too_far;
        ts *= 1.5; // ts will be divided by 1.5 in the next
        do
        {
          ts /= 1.5;
          point_set.clear();
          flag_too_far = false;
          Eigen::Vector3d last_pt = gl_traj.evaluate(0);
          for (t = 0; t < time; t += ts)
          {
            Eigen::Vector3d pt = gl_traj.evaluate(t);
            if ((last_pt - pt).norm() > cfg_.ctrl_pt_dist * 1.5)
            {
              flag_too_far = true;
              break;
            }
            last_pt = pt;
            point_set.push_back(pt);
          }
        } while (flag_too_far || point_set.size() < 7); // To make sure the initial path has enough points.
        t -= ts;
        start_end_derivatives.push_back(gl_traj.evaluateVel(0));
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(gl_traj.evaluateAcc(0));
        start_end_derivatives.push_back(gl_traj.evaluateAcc(t));
      }
      else // Initial path generated from previous trajectory.
      {
        double t;
        double t_cur = time_now - local_data_.start_time_;
        if (t_cur < 0)
          t_cur = 0;

        vector<double> pseudo_arc_length;
        vector<Eigen::Vector3d> segment_point;
        pseudo_arc_length.push_back(0.0);
        for (t = t_cur; t < local_data_.duration_ + 1e-3; t += ts)
        {
          segment_point.push_back(local_data_.position_traj_.evaluateDeBoorT(t));
          if (t > t_cur)
          {
            pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
          }
        }
        t -= ts;

        double poly_time = (local_data_.position_traj_.evaluateDeBoorT(t) - local_target_pt).norm() / cfg_.max_vel * 2;
        if (poly_time > ts)
        {
          PolynomialTraj gl_traj = PolynomialTraj::one_segment_traj_gen(local_data_.position_traj_.evaluateDeBoorT(t),
                                                                        local_data_.velocity_traj_.evaluateDeBoorT(t),
                                                                        local_data_.acceleration_traj_.evaluateDeBoorT(t),
                                                                        local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), poly_time);

          for (t = ts; t < poly_time; t += ts)
          {
            if (!pseudo_arc_length.empty())
            {
              segment_point.push_back(gl_traj.evaluate(t));
              pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
            }
            else
            {
              std::cerr << "[EgoLocalPlanner] pseudo_arc_length is empty, return!" << std::endl;
              continous_failures_count_++;
              return false;
            }
          }
        }

        double sample_length = 0;
        double cps_dist = cfg_.ctrl_pt_dist * 1.5; // cps_dist will be divided by 1.5 in the next
        size_t id = 0;
        do
        {
          cps_dist /= 1.5;
          point_set.clear();
          sample_length = 0;
          id = 0;
          while ((id <= pseudo_arc_length.size() - 2) && sample_length <= pseudo_arc_length.back())
          {
            if (sample_length >= pseudo_arc_length[id] && sample_length < pseudo_arc_length[id + 1])
            {
              point_set.push_back((sample_length - pseudo_arc_length[id]) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id + 1] +
                                  (pseudo_arc_length[id + 1] - sample_length) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id]);
              sample_length += cps_dist;
            }
            else
              id++;
          }
          point_set.push_back(local_target_pt);
        } while (point_set.size() < 7); // If the start point is very close to end point, this will help

        start_end_derivatives.push_back(local_data_.velocity_traj_.evaluateDeBoorT(t_cur));
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(local_data_.acceleration_traj_.evaluateDeBoorT(t_cur));
        start_end_derivatives.push_back(Eigen::Vector3d::Zero());

        if (point_set.size() > cfg_.planning_horizon / cfg_.ctrl_pt_dist * 3) // The initial path is unnormally too long!
        {
          flag_force_polynomial_ = true;
          flag_regenerate = true;
        }
      }
    } while (flag_regenerate);

    Eigen::MatrixXd ctrl_pts;
    UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);

    bspline_optimizer_->initControlPoints(ctrl_pts, true);

    t_init = std::chrono::steady_clock::now() - t_start;
    t_start = std::chrono::steady_clock::now();

    /*** STEP 2: OPTIMIZE ***/
    bool flag_step_1_success = bspline_optimizer_->BsplineOptimizeTrajRebound(ctrl_pts, ts);
    t_opt = std::chrono::steady_clock::now() - t_start;

    if (!flag_step_1_success)
    {
      continous_failures_count_++;
      return false;
    }

    t_start = std::chrono::steady_clock::now();

    UniformBspline pos = UniformBspline(ctrl_pts, 3, ts);
    pos.setPhysicalLimits(cfg_.max_vel, cfg_.max_acc, cfg_.feasibility_tolerance);

    /*** STEP 3: REFINE(RE-ALLOCATE TIME) IF NECESSARY ***/
    double ratio;
    bool flag_step_2_success = true;
    if (!pos.checkFeasibility(ratio, false))
    {
      std::cout << "[EgoLocalPlanner] Need to reallocate time." << std::endl;

      Eigen::MatrixXd optimal_control_points;
      flag_step_2_success = refineTrajAlgo(pos, start_end_derivatives, ratio, ts, optimal_control_points);
      if (flag_step_2_success)
        pos = UniformBspline(optimal_control_points, 3, ts);
    }

    if (!flag_step_2_success)
    {
      std::cerr << "[EgoLocalPlanner] This refined trajectory hits obstacles. It doesn't matter if it appears occasionally." << std::endl;
      continous_failures_count_++;
      return false;
    }

    t_refine = std::chrono::steady_clock::now() - t_start;

    // save planned results
    updateTrajInfo(pos, time_now);

    std::cout << "[EgoLocalPlanner] plan ok. init=" << t_init.count() * 1000
              << "ms, opt=" << t_opt.count() * 1000
              << "ms, refine=" << t_refine.count() * 1000 << "ms" << std::endl;

    continous_failures_count_ = 0;
    return true;
  }

  bool EgoLocalPlanner::refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts,
                                       Eigen::MatrixXd &optimal_control_points)
  {
    double t_inc;

    Eigen::MatrixXd ctrl_pts;

    reparamBspline(traj, start_end_derivative, ratio, ctrl_pts, ts, t_inc);

    traj = UniformBspline(ctrl_pts, 3, ts);

    double t_step = traj.getTimeSum() / (ctrl_pts.cols() - 3);
    bspline_optimizer_->ref_pts_.clear();
    for (double t = 0; t < traj.getTimeSum() + 1e-4; t += t_step)
      bspline_optimizer_->ref_pts_.push_back(traj.evaluateDeBoorT(t));

    bool success = bspline_optimizer_->BsplineOptimizeTrajRefine(ctrl_pts, ts, optimal_control_points);

    return success;
  }

  void EgoLocalPlanner::updateTrajInfo(const UniformBspline &position_traj, double time_now)
  {
    local_data_.start_time_ = time_now;
    local_data_.position_traj_ = position_traj;
    local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();
    local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
    local_data_.start_pos_ = local_data_.position_traj_.evaluateDeBoorT(0.0);
    local_data_.duration_ = local_data_.position_traj_.getTimeSum();
    local_data_.traj_id_ += 1;
  }

  void EgoLocalPlanner::reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio,
                                       Eigen::MatrixXd &ctrl_pts, double &dt, double &time_inc)
  {
    double time_origin = bspline.getTimeSum();
    int seg_num = bspline.getControlPoint().cols() - 3;

    bspline.lengthenTime(ratio);
    double duration = bspline.getTimeSum();
    dt = duration / double(seg_num);
    time_inc = duration - time_origin;

    vector<Eigen::Vector3d> point_set;
    for (double time = 0.0; time <= duration + 1e-4; time += dt)
    {
      point_set.push_back(bspline.evaluateDeBoorT(time));
    }
    UniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
  }

} // namespace ego_planner
