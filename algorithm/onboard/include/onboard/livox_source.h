#ifndef ONBOARD_LIVOX_SOURCE_H
#define ONBOARD_LIVOX_SOURCE_H
#include <deque>
#include <mutex>
#include <string>
#include <livox_lidar_api.h>
#include <sensor/frame_assembler.h>
namespace ego_onboard {
class LivoxSource {
 public:
  struct Config {
    bool allow_zero_crc = false;
    std::string expected_serial;
    int point_data_type = 1;
    double frame_duration_s = 0.1, gap_factor = 2.0;
    double min_range_m = 0.1, max_range_m = 100.0;
    size_t queue_capacity = 3;
  };
  struct Stats { uint64_t packets=0, invalid=0, frames=0, queue_drops=0, crc_missing=0; uint8_t time_type=0; double last_rx=0; uint8_t sync_type=0; uint64_t last_sync_ns=0; int64_t time_offset_ns=0; double state_rx=0; std::string serial, ip; };
  ~LivoxSource();
  bool start(const std::string &config, const std::string &ip, std::string &error, bool diagnose_zero_crc=false, const Config *settings=nullptr);
  void stop();
  bool pop(ego_sensor::LidarFrame &frame);
  Stats stats() const;
 private:
  static void state(uint32_t, uint8_t, const char *, void *);
  static void points(uint32_t, uint8_t, LivoxLidarEthernetPacket *, void *);
  static void info(uint32_t, const LivoxLidarInfo *, void *);
  static void dataType(livox_status, uint32_t, LivoxLidarAsyncControlResponse *, void *);
  static void mode(livox_status, uint32_t, LivoxLidarAsyncControlResponse *, void *);
  mutable std::mutex mutex_;
  ego_sensor::FrameAssembler assembler_;
  std::deque<ego_sensor::LidarFrame> queue_;
  std::string wanted_ip_;
  uint32_t handle_ = 0;
  bool selected_ = false, started_ = false;
  Stats stats_;
  Config config_;
};
}
#endif
