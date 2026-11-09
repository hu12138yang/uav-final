#include <onboard/livox_source.h>
#include <onboard/time_alignment.h>
#include <cstdio>
#include <rapidjson/document.h>
#include <data_handler/data_handler.h>
namespace ego_onboard {
LivoxSource::~LivoxSource() { stop(); }
bool LivoxSource::start(const std::string &config, const std::string &ip, std::string &error, bool diagnose_zero_crc, const Config *settings) {
  config_ = settings ? *settings : Config();
  if (config_.allow_zero_crc && (ip.empty() || config_.expected_serial.empty() || config_.point_data_type != 1)) {
    error="Mid360S sample CRC profile requires an explicit IP, serial number and Cartesian High data"; return false;
  }
  assembler_ = ego_sensor::FrameAssembler();
  ego_sensor::FrameAssembler::Config frame; frame.frame_duration=config_.frame_duration_s; frame.gap_factor=config_.gap_factor;
  assembler_.setConfig(frame);
  ego_sensor::LivoxParser::Config parser;parser.min_range=config_.min_range_m;parser.max_range=config_.max_range_m;
  assembler_.setParserConfig(parser);
  livox::lidar::DataHandler::GetInstance().AllowZeroMid360sPointCrc(diagnose_zero_crc || config_.allow_zero_crc);
  wanted_ip_ = ip;
  if (!LivoxLidarSdkInit(config.c_str())) {
    LivoxLidarSdkUninit(); error = "SDK2 initialization failed: check host_ip/local interface/UDP ports"; return false;
  }
  started_ = true;
  SetLivoxLidarPointCloudCallBack(points, this);
  SetLivoxLidarInfoCallback(state, this);
  SetLivoxLidarInfoChangeCallback(info, this);
  return true;
}
void LivoxSource::stop() {
  if (started_) { LivoxLidarSdkUninit(); started_ = false; }
  livox::lidar::DataHandler::GetInstance().AllowZeroMid360sPointCrc(false);
}
void LivoxSource::dataType(livox_status s, uint32_t h, LivoxLidarAsyncControlResponse *r, void *) {
  std::fprintf(stderr,"[Livox] Cartesian High handle=%u status=%d ret=%d\n",h,int(s),r?int(r->ret_code):-1);
}
void LivoxSource::mode(livox_status s, uint32_t h, LivoxLidarAsyncControlResponse *r, void *) {
  std::fprintf(stderr, "[Livox] NORMAL handle=%u status=%d ret=%d\n", h, int(s), r ? int(r->ret_code) : -1);
}
void LivoxSource::info(uint32_t h, const LivoxLidarInfo *i, void *ctx) {
  auto &self = *static_cast<LivoxSource *>(ctx);
  if (!i || i->dev_type != kLivoxLidarTypeMid360s) return;
  {
    std::lock_guard<std::mutex> lock(self.mutex_);
    if ((!self.config_.expected_serial.empty() && self.config_.expected_serial != i->sn) || (!self.wanted_ip_.empty() && self.wanted_ip_ != i->lidar_ip) || (self.selected_ && self.handle_ != h)) return;
    self.handle_ = h; self.selected_ = true;
    self.stats_.serial = i->sn; self.stats_.ip = i->lidar_ip;
  }
  std::printf("[Livox] Mid360S SN=%s IP=%s\n", i->sn, i->lidar_ip);
  SetLivoxLidarWorkMode(h, kLivoxLidarNormal, mode, ctx);
  SetLivoxLidarPclDataType(h, static_cast<LivoxLidarPointDataType>(self.config_.point_data_type), dataType, ctx);
}
void LivoxSource::state(uint32_t h, uint8_t type, const char *json, void *ctx) {
  if(!json || type!=kLivoxLidarTypeMid360s) return;
  rapidjson::Document d; d.Parse(json);
  if(d.HasParseError()||!d.IsObject()) return;
  auto &self=*static_cast<LivoxSource*>(ctx);
  std::lock_guard<std::mutex> lock(self.mutex_);
  if(!self.selected_||self.handle_!=h) return;
  if(d.HasMember("time_sync_type")&&d["time_sync_type"].IsUint()) self.stats_.sync_type=d["time_sync_type"].GetUint();
  if(d.HasMember("last_sync_time")&&d["last_sync_time"].IsUint64()) {
    self.stats_.last_sync_ns=d["last_sync_time"].GetUint64(); self.stats_.state_rx=monotonicSeconds();
  }
  if(d.HasMember("time_offset")&&d["time_offset"].IsInt64()) self.stats_.time_offset_ns=d["time_offset"].GetInt64();
}
void LivoxSource::points(uint32_t h, uint8_t type, LivoxLidarEthernetPacket *p, void *ctx) {
  auto &self = *static_cast<LivoxSource *>(ctx);
  std::lock_guard<std::mutex> lock(self.mutex_);
  if (!self.selected_ || self.handle_ != h || type != kLivoxLidarTypeMid360s || !p) return;
  ++self.stats_.packets; self.stats_.time_type = p->time_type; self.stats_.last_rx = monotonicSeconds();
  // SDK validates datagram boundaries and any nonzero CRC; selected-device profile may omit CRC.
  if (p->data_type != self.config_.point_data_type) { ++self.stats_.invalid; return; }
  if (p->crc32 == 0) ++self.stats_.crc_missing;
  const auto dropped = self.assembler_.droppedPackets();
  self.assembler_.addPacket(*p, 0);
  self.stats_.invalid += self.assembler_.droppedPackets() - dropped;
  ego_sensor::LidarFrame frame;
  if (self.assembler_.popFrame(frame)) {
    if (self.queue_.size() >= self.config_.queue_capacity) { self.queue_.pop_front(); ++self.stats_.queue_drops; }
    self.queue_.push_back(std::move(frame)); ++self.stats_.frames;
  }
}
bool LivoxSource::pop(ego_sensor::LidarFrame &f) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (queue_.empty()) return false;
  f = std::move(queue_.front()); queue_.pop_front(); return true;
}
LivoxSource::Stats LivoxSource::stats() const { std::lock_guard<std::mutex> lock(mutex_); return stats_; }
}
