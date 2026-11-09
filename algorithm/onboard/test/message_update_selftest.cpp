// Verify continuous messages, independent snapshots and timestamp update rules.
// Only in-memory frames are used; this test never opens a device.
#include <onboard/offboard_pilot.h>
#include <cstdio>

using namespace ego_mav;
using namespace ego_onboard;
namespace {
int failures = 0;
void check(bool ok, const char *label) {
  std::printf("[%s] %s\n", ok ? "PASS" : "FAIL", label);
  if (!ok) ++failures;
}
struct History : PlannerIO {
  std::vector<ego_sensor::TimedPose> poses;
  void onPose(const ego_sensor::TimedPose &p) override { poses.push_back(p); }
  void onScanFrame(const ego_sensor::LidarFrame &) override {}
  bool step(double, const Eigen::Vector3d &, ego_planner::QuadrotorCommand &) override { return false; }
};
std::vector<uint8_t> position(uint32_t boot, int32_t height, uint8_t source=1) {
  uint8_t p[28]{};
  wrU32(p, 0, boot); wrI32(p, 4, 210000000); wrI32(p, 8, 1130000000);
  wrI32(p, 16, height);
  return pack(source, 1, uint8_t(boot), 33, p, sizeof(p));
}
std::vector<uint8_t> attitude(uint32_t boot) {
  uint8_t p[28]{}; wrU32(p, 0, boot);
  return pack(1, 1, uint8_t(boot), 30, p, sizeof(p));
}
void append(std::vector<uint8_t> &stream, const std::vector<uint8_t> &frame) {
  stream.insert(stream.end(), frame.begin(), frame.end());
}
void feed(OffboardPilot &pilot, double now, const std::vector<uint8_t> &frame) {
  pilot.setReceiveTime(now); pilot.onMavlinkBytes(frame.data(), frame.size());
}
}
int main() {
  OffboardPilot pilot;
  History history;
  pilot.setPlanner(&history);
  OffboardPilot::Config config;
  config.require_timesync = false; // Isolate message/storage rules from clock estimation.
  pilot.setConfig(config); pilot.startObserving();
  uint8_t home[60]{}; wrI32(home, 0, 210000000); wrI32(home, 4, 1130000000);
  feed(pilot, 10, pack(1, 1, 0, 242, home, sizeof(home)));
  std::vector<uint8_t> stream;
  for (uint32_t i=0; i<100; ++i) {
    append(stream, position(1000+i*10, 50000+int32_t(i)*10));
    append(stream, attitude(1000+i*10));
  }
  // One receive chunk deliberately contains 200 adjacent complete messages.
  feed(pilot, 10.1, stream);
  check(history.poses.size() == 100, "200 adjacent messages are all parsed, without replacement or concatenation damage");
  bool ordered = history.poses.size() == 100;
  for (size_t i=0; i<history.poses.size(); ++i)
    ordered = ordered && std::abs(history.poses[i].p.z()-(50+i*.01)) < 1e-9;
  check(ordered && std::abs(pilot.latestPose().p.z()-50.99) < 1e-9,
        "current pose is the newest sample while all earlier pose copies retain their values");
  const auto saved = pilot.latestPose();
  feed(pilot, 10.11, position(1990, 90000));
  feed(pilot, 10.11, attitude(1990));
  feed(pilot, 10.12, position(1980, 80000));
  feed(pilot, 10.12, attitude(1980));
  check(history.poses.size() == 100 && (pilot.latestPose().p-saved.p).norm() == 0,
        "duplicate and slightly out-of-order position/attitude cannot overwrite newer state");
  feed(pilot, 10.13, position(2000, 70000, 2));
  check((pilot.latestPose().p-saved.p).norm() == 0,
        "another aircraft's message cannot replace this aircraft's position");
  auto damaged = position(2000, 70000); damaged[14] ^= 1;
  feed(pilot, 10.14, damaged);
  check(pilot.crcErrors() > 0 && (pilot.latestPose().p-saved.p).norm() == 0,
        "CRC-damaged message cannot replace the valid state");
  feed(pilot, 10.15, position(2000, 51000));
  feed(pilot, 10.15, attitude(2000));
  check(history.poses.size() == 101 && std::abs(pilot.latestPose().p.z()-51) < 1e-9 &&
        std::abs(history.poses.front().p.z()-50) < 1e-9,
        "next valid update recovers and leaves previous samples intact");
  feed(pilot, 10.16, position(1000, 10000));
  check(pilot.phase() == Phase::FAILED && std::abs(pilot.latestPose().p.z()-51) < 1e-9,
        "large boot-time rollback fails explicitly instead of overwriting with an old position");

  ego_sensor::PoseBuffer poses;
  poses.setMaxSize(3);
  ego_sensor::TimedPose pose;
  for (int i=0; i<4; ++i) { pose.t=i; pose.p.x()=i; poses.add(pose); }
  ego_sensor::TimedPose out;
  check(!poses.poseAt(0, out) && poses.poseAt(3, out) && out.p.x()==3,
        "bounded pose history evicts oldest entries and retains newest entries");
  pose.t=2; pose.p.x()=99; poses.add(pose);
  check(poses.poseAt(3, out) && out.p.x()==3, "older pose insertion does not replace the newest history entry");
  pose.t=3; pose.p.x()=4; poses.add(pose);
  check(poses.poseAt(3, out) && out.p.x()==4, "same-time pose correction deliberately replaces its matching history entry");
  std::printf("message_update_selftest: %d failures\n", failures);
  return failures ? 1 : 0;
}
