#ifndef ONBOARD_ASYNC_PLANNER_H
#define ONBOARD_ASYNC_PLANNER_H
#include <onboard/ego_planner_adapter.h>
#include <onboard/time_alignment.h>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <thread>

namespace ego_onboard {
// Control never waits on map insertion or optimization. Bounded input queues,
// expiring output commands, and map freshness form the worker watchdog.
class AsyncPlanner : public PlannerIO {
 public:
  AsyncPlanner(const EgoPlannerAdapterConfig &cfg, double ref) : cfg_(cfg), ref_(ref) {
    worker_ = std::thread([this]{
      try { run(); }
      catch(const std::exception &e) {
        std::lock_guard<std::mutex> lock(m_); healthy_=false; have_cmd_=false;
        std::fprintf(stderr,"[PLANNER] worker stopped: %s\n",e.what());
      }
    });
  }
  ~AsyncPlanner() { running_ = false; wake_.notify_all(); worker_.join(); }
  void setLeg(const Eigen::Vector3d &a, const Eigen::Vector3d &b, bool last) override {
    std::lock_guard<std::mutex> l(m_); a_=a; b_=b; last_=last; leg_=true; have_cmd_=false; ++generation_; wake_.notify_one();
  }
  void onPose(const ego_sensor::TimedPose &p) override {
    std::lock_guard<std::mutex> l(m_); poses_.push_back(p); latest_=p;
    if (poses_.size() > 500) poses_.pop_front(); wake_.notify_one();
  }
  void onScanFrame(const ego_sensor::LidarFrame &f) override {
    std::lock_guard<std::mutex> l(m_);
    if (scans_.size() >= 3) scans_.pop_front(); scans_.push_back(f); wake_.notify_one();
  }
  bool step(double now, const Eigen::Vector3d &p, ego_planner::QuadrotorCommand &cmd) override {
    std::lock_guard<std::mutex> l(m_); requested_=now; wake_.notify_one();
    if (have_cmd_ && now - command_time_ <= 0.15) { cmd=command_; return true; }
    cmd.position=p; cmd.velocity.setZero(); cmd.acceleration.setZero(); cmd.yaw=0; cmd.yaw_dot=0;
    return false;
  }
  void dropTrajectory() override {
    std::lock_guard<std::mutex> l(m_); drop_=true; have_cmd_=false; requested_=-1e9; ++generation_;
  }
  bool sensingHealthy() const override {
    std::lock_guard<std::mutex> l(m_);
    return healthy_ && monotonicSeconds() - last_map_ <= cfg_.sensing_timeout_s &&
           monotonicSeconds() - last_worker_ <= 0.25;
  }
  EgoPlannerAdapter::Stats stats() const { std::lock_guard<std::mutex> l(m_); return stats_; }
 private:
  void run() {
    EgoPlannerAdapter adapter; adapter.init(cfg_); adapter.prepareMap(ref_);
    std::deque<ego_sensor::LidarFrame> pending;
    while (running_) {
      std::deque<ego_sensor::TimedPose> poses;
      std::deque<ego_sensor::LidarFrame> scans;
      Eigen::Vector3d a,b; bool leg,drop,last; double request; ego_sensor::TimedPose latest;
      uint64_t generation;
      {
        std::unique_lock<std::mutex> l(m_);
        wake_.wait_for(l, std::chrono::milliseconds(20));
        poses.swap(poses_); scans.swap(scans_); a=a_; b=b_; last=last_; leg=leg_; leg_=false;
        drop=drop_; drop_=false; request=requested_; latest=latest_; generation=generation_;
      }
      if (leg) adapter.setLeg(a,b,last);
      if (drop) adapter.dropTrajectory();
      for (const auto &p:poses) adapter.onPose(p);
      for (auto &f:scans) pending.push_back(std::move(f));
      while (pending.size() > 3) pending.pop_front();
      const double now=monotonicSeconds();
      while (!pending.empty()) {
        auto &f=pending.front();
        if (now - f.t_base > cfg_.sensing_timeout_s) { pending.pop_front(); continue; }
        if (!adapter.frameCovered(f)) break; // Wait for FCU pose bracket at scan end.
        adapter.onScanFrame(f); pending.pop_front();
      }
      ego_planner::QuadrotorCommand cmd;
      const bool ok=now-request < 0.1 && adapter.sensingHealthy() && adapter.step(now,latest.p,cmd);
      {
        std::lock_guard<std::mutex> l(m_);
        const auto st=adapter.stats();
        if (st.inflate_samples > stats_.inflate_samples) last_map_=monotonicSeconds();
        stats_=st; healthy_=adapter.sensingHealthy(); last_worker_=monotonicSeconds();
        if (ok && generation==generation_) { command_=cmd; command_time_=now; have_cmd_=true; }
      }
    }
  }
  EgoPlannerAdapterConfig cfg_; double ref_;
  mutable std::mutex m_; std::condition_variable wake_; std::thread worker_; std::atomic<bool> running_{true};
  std::deque<ego_sensor::TimedPose> poses_; std::deque<ego_sensor::LidarFrame> scans_;
  ego_sensor::TimedPose latest_;
  Eigen::Vector3d a_=Eigen::Vector3d::Zero(), b_=Eigen::Vector3d::Zero();
  bool last_=false, leg_=false, drop_=false, healthy_=false, have_cmd_=false;
  double requested_=-1e9, command_time_=-1e9, last_map_=-1e9, last_worker_=-1e9;
  uint64_t generation_=0;
  ego_planner::QuadrotorCommand command_; EgoPlannerAdapter::Stats stats_;
};
}
#endif
