#include "sensor/frame_assembler.h"
#include <algorithm>
#include <utility>

namespace ego_sensor {
void FrameAssembler::addPacket(const LivoxLidarEthernetPacket &pkt, double offset) {
  const uint64_t ns = LivoxParser::packetTimeNs(pkt);
  const uint64_t span = uint64_t(pkt.time_interval) * 100ull;
  // A gap, clock step, restart, or sync-source change invalidates the partial scan.
  // Keep the first valid packet after the discontinuity to permit recovery.
  if (have_last_ && (ns <= last_pkt_ns_ || pkt.time_type != last_type_ ||
      ns - last_pkt_ns_ > std::max<uint64_t>(100000, uint64_t(cfg_.gap_factor * last_span_ns_)))) {
    ++dropped_;
    cur_ = LidarFrame(); in_frame_ = complete_ = false;
  }
  if (!in_frame_) {
    cur_ = LidarFrame(); cur_.lidar_id = lidar_id_; cur_.time_type = pkt.time_type;
    t_base_ns_ = ns;
  }
  if (!parser_.parsePacket(pkt, offset, cur_)) { ++dropped_; return; }
  cur_.payload_crc_present = cur_.payload_crc_present && pkt.crc32 != 0;
  in_frame_ = true;
  last_pkt_ns_ = ns; last_span_ns_ = span; last_type_ = pkt.time_type; have_last_ = true;
  if (ns - t_base_ns_ + span >= frame_duration_ns_) complete_ = true;
  if (cur_.points.size() > 100000) { ++dropped_; reset(); }
}
bool FrameAssembler::popFrame(LidarFrame &out) {
  if (!complete_) return false;
  out = std::move(cur_); cur_ = LidarFrame(); in_frame_ = complete_ = false;
  ++frames_out_; return true;
}
void FrameAssembler::flush(LidarFrame &out) {
  if (cur_.points.empty()) return;
  out = std::move(cur_); cur_ = LidarFrame(); in_frame_ = complete_ = false; ++frames_out_;
}
void FrameAssembler::reset() {
  cur_ = LidarFrame(); in_frame_ = complete_ = have_last_ = false;
  last_pkt_ns_ = last_span_ns_ = 0;
}
}
