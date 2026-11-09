// Hardware-free integration: recorded Mid360S payload -> SDK boundary -> real
// parser/frame assembler -> epoch conversion -> strict deskew -> map/worker.
// The payload is measured; repetition, timestamps and poses are test fixtures.
#include <data_handler/data_handler.h>
#include <onboard/async_planner.h>
#include <onboard/config_file.h>
#include <sensor/frame_assembler.h>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iterator>
#include <thread>

namespace {
int failures = 0;
void check(bool ok, const char *label) {
  std::printf("[%s] %s\n", ok ? "PASS" : "FAIL", label);
  if (!ok) ++failures;
}
struct Capture {
  ego_sensor::FrameAssembler assembler;
  std::vector<ego_sensor::LidarFrame> frames;
  int packets = 0;
  static void receive(uint32_t, uint8_t, LivoxLidarEthernetPacket *packet, void *context) {
    auto &self = *static_cast<Capture *>(context);
    ++self.packets;
    self.assembler.addPacket(*packet, 0);
    ego_sensor::LidarFrame frame;
    while (self.assembler.popFrame(frame)) self.frames.push_back(std::move(frame));
  }
};
ego_sensor::TimedPose testPose(double time, double base) {
  ego_sensor::TimedPose pose;
  pose.t = time;
  pose.p = Eigen::Vector3d(1 + 0.2 * (time-base), 2, 50);
  pose.q = Eigen::Quaterniond(Eigen::AngleAxisd(ego_mav::kPi/2, Eigen::Vector3d::UnitZ()));
  return pose;
}
template<class Predicate> bool waitUntil(Predicate predicate) {
  const double deadline = ego_onboard::monotonicSeconds() + 1;
  while (!predicate()) {
    if (ego_onboard::monotonicSeconds() >= deadline) return false;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return true;
}
}

int main(int argc, char **argv) {
  if (argc != 3) return 2;
  ego_onboard::AppConfig app;
  std::string error;
  if (!ego_onboard::loadConfigFile(argv[2], app, error)) {
    std::fprintf(stderr, "%s\n", error.c_str()); return 2;
  }
  std::ifstream input(argv[1], std::ios::binary);
  std::vector<uint8_t> packet((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
  check(packet.size() == 1380 && packet[10] == 1 && packet[5] == 96,
        "load measured Cartesian High 96-point packet");
  if (failures) return 1;
  Capture capture;
  ego_sensor::FrameAssembler::Config assembly;
  assembly.frame_duration = app.lidar.frame_duration_s;
  assembly.gap_factor = app.lidar.gap_factor;
  capture.assembler.setConfig(assembly);
  ego_sensor::LivoxParser::Config parser;
  parser.min_range = app.lidar.min_range_m;
  parser.max_range = app.lidar.max_range_m;
  capture.assembler.setParserConfig(parser);
  auto &sdk = livox::lidar::DataHandler::GetInstance();
  sdk.SetPointDataCallback(Capture::receive, &capture);
  sdk.AllowZeroMid360sPointCrc(false);
  sdk.Handle(35, 1, packet.data(), packet.size());
  check(capture.packets == 0, "strict CRC policy rejects the recorded absent CRC");
  sdk.AllowZeroMid360sPointCrc(app.lidar.allow_zero_crc);
  uint64_t initial_ns = 0;
  std::memcpy(&initial_ns, packet.data()+28, sizeof(initial_ns));
  for (uint64_t i = 0; i < 250; ++i) {
    const uint64_t stamp = initial_ns + i * 500000;
    std::memcpy(packet.data()+28, &stamp, sizeof(stamp));
    sdk.Handle(35, 1, packet.data(), packet.size());
  }
  check(capture.packets == 250 && capture.frames.size() == 1,
        "production compatibility policy feeds real parser and 100 ms assembler");
  auto malformed = packet;
  malformed[24] = 1;
  sdk.Handle(35, 1, malformed.data(), malformed.size());
  sdk.Handle(35, 1, packet.data(), 35);
  check(capture.packets == 250, "nonzero bad CRC and truncated packets never reach the frame assembler");
  if (capture.frames.empty() || capture.frames.front().points.empty()) return 1;
  const double saved_base = capture.frames.front().t_base;
  const Eigen::Vector3d saved_point = capture.frames.front().points.front().p;
  for (uint64_t i = 250; i < 650; ++i) {
    const uint64_t stamp = initial_ns + i * 500000;
    std::memcpy(packet.data()+28, &stamp, sizeof(stamp));
    sdk.Handle(35, 1, packet.data(), packet.size());
  }
  check(capture.frames.size() == 3 && capture.frames.front().t_base == saved_base &&
        (capture.frames.front().points.front().p-saved_point).norm() == 0 &&
        capture.frames[2].t_base > capture.frames[1].t_base && capture.frames[1].t_base > saved_base,
        "successive point-cloud frames own independent data; a new frame never rewrites an older frame");
  sdk.AllowZeroMid360sPointCrc(false);
  sdk.Destory();
  if (capture.frames.empty()) return 1;
  auto frame = capture.frames.front();
  check(!frame.payload_crc_present && frame.time_type == 1 && !frame.points.empty(),
        "frame preserves absent CRC, PTP type and measured valid points");
  // Independent little-endian integer oracle for the first surviving raw point.
  Eigen::Vector3d first = Eigen::Vector3d::Zero();
  bool found = false;
  for (int i = 0; i < 96 && !found; ++i) {
    int32_t xyz[3]; std::memcpy(xyz, packet.data()+36+i*14, 12);
    const Eigen::Vector3d p(xyz[0]*0.001, xyz[1]*0.001, xyz[2]*0.001);
    if (p.norm() >= parser.min_range && p.norm() <= parser.max_range) { first=p; found=true; }
  }
  check(found && (frame.points.front().p-first).norm() < 1e-9,
        "wire millimeters become sensor-frame meters and zero returns are filtered");
  check(frame.points.back().t_rel >= .099 && frame.points.back().t_rel <= .101,
        "assembled points retain their individual sample times over 100 ms");

  // Rebase the recorded PTP epoch onto a controlled monotonic axis. No actual
  // PTP accuracy or physical extrinsic calibration is claimed by this fixture.
  const double base = ego_onboard::monotonicSeconds() - .15;
  ego_onboard::EpochBridge epoch;
  epoch.sample(base, frame.t_base);
  frame.t_base = epoch.toMonotonic(frame.t_base);
  check(std::abs(frame.t_base-base) < 1e-6, "recorded epoch converts to the common monotonic axis");
  auto cfg = app.adapter;
  cfg.strict_pose_coverage = true; // Same strict setting as the physical main.
  ego_onboard::EgoPlannerAdapter adapter;
  adapter.init(cfg); adapter.prepareMap(50);
  adapter.onPose(testPose(base-.01, base));
  adapter.onScanFrame(frame);
  check(adapter.stats().frames_dropped == 1 && adapter.stats().inflate_samples == 0,
        "missing pose brackets reject the whole frame without updating the map");
  ego_sensor::PoseBuffer poses;
  poses.setStrictCoverage(true);
  for (int i = -1; i <= 14; ++i) {
    const auto pose = testPose(base+i*.01, base);
    adapter.onPose(pose); poses.add(pose);
  }
  std::vector<Eigen::Vector3d> world;
  check(adapter.frameCovered(frame) && adapter.scanAssembler().toWorld(frame, poses, world),
        "complete pose brackets permit strict per-point deskew");
  double maximum_error = 0;
  for (size_t i = 0; i < world.size(); ++i) {
    const auto &p = frame.points[i];
    const auto pose = testPose(frame.t_base+p.t_rel, base);
    const Eigen::Vector3d body = cfg.extrinsic.R*p.p + cfg.extrinsic.t;
    // Independent +90 degree yaw rotation and linear motion, no production
    // quaternion conversion in the expected world-coordinate calculation.
    const Eigen::Vector3d expected = pose.p + Eigen::Vector3d(-body.y(), body.x(), body.z());
    maximum_error = std::max(maximum_error, (world[i]-expected).norm());
  }
  check(world.size() == frame.points.size() && maximum_error < 1e-8,
        "all world points match independent rotation, translation and sample-time motion");
  adapter.onScanFrame(frame);
  size_t occupied = 0;
  for (const auto &p : world) if (adapter.gridMap()->getInflateOccupancy(p) == 1) ++occupied;
  check(adapter.stats().inflate_samples == 1 && adapter.stats().cloud_points == long(world.size()) &&
        occupied > 0 && adapter.sensingHealthy(),
        "measured points enter the actual EGO occupancy map and mark obstacles");
  ego_planner::QuadrotorCommand command;
  check(adapter.step(base+.14, testPose(base+.14, base).p, command) &&
        command.position.allFinite() && command.velocity.norm() == 0,
        "mapped cloud is consumable by the planner; no-route case produces HOLD");
  std::printf("[REPLAY] packets=%d frames=%zu points=%zu occupied_points=%zu world_error_m=%.3g\n",
              capture.packets, capture.frames.size(), world.size(), occupied, maximum_error);

  // Exercise the actual bounded asynchronous worker used by onboard_main.
  // Rebase again so build/diagnostic work cannot age the scan before enqueue.
  const double async_base = ego_onboard::monotonicSeconds() - .12;
  frame.t_base = async_base;
  ego_onboard::AsyncPlanner worker(cfg, 50);
  worker.onScanFrame(frame);
  std::this_thread::sleep_for(std::chrono::milliseconds(40));
  check(worker.stats().inflate_samples == 0, "async worker waits for real pose coverage before map insertion");
  for (int i = -1; i <= 14; ++i) worker.onPose(testPose(async_base+i*.01, async_base));
  check(waitUntil([&]{ return worker.stats().inflate_samples == 1; }) &&
        worker.stats().cloud_points == long(frame.points.size()) && worker.sensingHealthy(),
        "same measured frame reaches the production async map worker once poses arrive");
  std::this_thread::sleep_for(std::chrono::milliseconds(int(cfg.sensing_timeout_s*1000)+100));
  check(!worker.sensingHealthy(), "point-cloud interruption expires async map freshness");
  std::printf("lidar_system_replay_selftest: %d failures (offline fixture, no hardware IO)\n", failures);
  return failures ? 1 : 0;
}
