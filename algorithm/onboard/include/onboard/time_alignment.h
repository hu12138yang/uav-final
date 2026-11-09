#ifndef ONBOARD_TIME_ALIGNMENT_H
#define ONBOARD_TIME_ALIGNMENT_H
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <chrono>

namespace ego_onboard {
inline double monotonicSeconds() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
inline double unixSeconds() {
  return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
}
// TIMESYNC mid-point estimator. Receive timestamps are NEVER used as sample times.
// RTT/2 is an estimated uncertainty bound, assuming a responsive peer clock.
class TimeAlignment {
 public:
  double max_rtt_s = 0.010, max_residual_s = 0.005, timeout_s = 3.0;
  int min_samples = 10;
  int64_t request(double now) {
    pending_ = int64_t(now * 1e9); sent_ = now; return pending_;
  }
  bool accept(int64_t tc1, int64_t ts1, double now) {
    if (!pending_ || ts1 != pending_ || tc1 <= 0) { ++rejected_; return false; }
    pending_ = 0;
    const double rtt = now - sent_;
    if (rtt < 0 || rtt > max_rtt_s) { ++rejected_; return false; }
    if (samples_ && now-last_ > timeout_s) reset();
    const double measured = (sent_ + now) * 0.5 - double(tc1) * 1e-9;
    if (samples_ && std::abs(measured - offset_) > max_residual_s) {
      reset(); ++rejected_; return false;
    }
    residual_ = samples_ ? std::abs(measured - offset_) : 0;
    offset_ = samples_ ? offset_ * 0.9 + measured * 0.1 : measured;
    rtt_ = rtt; last_ = now; ++samples_; return true;
  }
  bool valid(double now) const { return samples_ >= min_samples && now >= last_ && now - last_ <= timeout_s; }
  double hostTime(double boot_seconds) const { return boot_seconds + offset_; }
  uint32_t bootMs(double now) const { return uint32_t(uint64_t(std::llround(std::max(0.0, (now - offset_) * 1000)))); }
  double offset() const { return offset_; }
  double rtt() const { return rtt_; }
  double residual() const { return residual_; }
  int samples() const { return samples_; }
  uint64_t rejected() const { return rejected_; }
  uint64_t generation() const { return generation_; }
  void reset() { samples_ = 0; pending_ = 0; last_ = -1e9; ++generation_; }
 private:
  int64_t pending_ = 0;
  double sent_ = 0, offset_ = 0, last_ = -1e9, rtt_ = 0, residual_ = 0;
  int samples_ = 0;
  uint64_t rejected_ = 0, generation_ = 0;
};
// PTP master is CLOCK_REALTIME with ptpTimescale=0. Convert its Unix epoch
// to the host monotonic axis. Detect NTP/manual clock steps; reject and flush.
class EpochBridge {
 public:
  bool sample(double mono, double epoch) {
    const double next = epoch - mono;
    const bool stable = !have_ || std::abs(next - offset_) <= 0.005;
    offset_ = next; have_ = true; return stable;
  }
  double toMonotonic(double epoch) const { return epoch - offset_; }
 private:
  bool have_ = false;
  double offset_ = 0;
};
}
#endif
