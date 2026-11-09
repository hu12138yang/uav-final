#ifndef _SENSOR_FRAME_ASSEMBLER_H_
#define _SENSOR_FRAME_ASSEMBLER_H_

#include <cstdint>

#include <livox_lidar_def.h>

#include "sensor/livox_parser.h"

namespace ego_sensor
{

  // 把"一个个 UDP 包"攒成"一帧点云"。
  //
  // SDK 的回调一次只给一个包（最多 dot_num 个点），成帧必须自己做 ——
  // 参考 livox_ros_driver 的 Lddc::PublishCustomPointcloud：
  //   timebase    = 本帧第一个包的时间戳
  //   offset_time = 该点绝对时间 - timebase
  //   （老驱动用 product_type 常量表取 point_interval，这里用包自带的
  //     time_interval，对 Mid-360S 更准）
  // 发现包间大缺口、时间回跳或同步源变化时丢弃未完成的半帧，
  // 从当前有效包重新开始；不把旧时钟的点拼入新帧。
  class FrameAssembler
  {
  public:
    struct Config
    {
      double frame_duration = 0.1; // 出帧周期（10 Hz）。按"首包时刻 + duration"切帧
      double gap_factor = 2.0;     // 包间隔超过此倍数的包总跨度时清理半帧
    };

    void setConfig(const Config &c)
    {
      cfg_ = c;
      // 时间比较一律走整数纳秒，避免浮点边界（时间戳本身就是整数 ns）
      frame_duration_ns_ = (uint64_t)(c.frame_duration * 1e9 + 0.5);
    }
    void setParserConfig(const LivoxParser::Config &c) { parser_.setConfig(c); }
    void setLidarId(uint8_t id) { lidar_id_ = id; }

    // 追加一个包（直接来自 LivoxLidarPointCloudCallBack）。
    // t_epoch_offset: 雷达时间戳 -> 位姿时钟 的偏移，由时间同步环节给出。
    void addPacket(const LivoxLidarEthernetPacket &pkt, double t_epoch_offset);

    // 取走一帧；返回 false = 还没攒够一帧（调用方下个包再来）
    bool popFrame(LidarFrame &out);

    // 任务结束 / 主动冲刷剩余点
    void flush(LidarFrame &out);
    void reset();

    uint64_t droppedPackets() const { return dropped_; }
    uint64_t framesOut() const { return frames_out_; }

  private:
    bool have_last_ = false;
    uint8_t last_type_ = 0;
    Config cfg_;
    LivoxParser parser_;
    uint8_t lidar_id_ = 0;

    LidarFrame cur_;
    bool in_frame_ = false;
    bool complete_ = false;
    uint64_t t_base_ns_ = 0;         // 本帧首包的雷达时间戳
    uint64_t frame_duration_ns_ = 100000000ull;
    uint64_t last_pkt_ns_ = 0;
    uint64_t last_span_ns_ = 0;      // 上一包的理论时间跨度
    uint64_t dropped_ = 0;
    uint64_t frames_out_ = 0;
  };

} // namespace ego_sensor

#endif
