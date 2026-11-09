#ifndef _SENSOR_LIVOX_PARSER_H_
#define _SENSOR_LIVOX_PARSER_H_

// Livox-SDK2 的包/点结构定义。只 include 头文件、不必链接 SDK：
// livox_lidar_def.h 只依赖 <stdint.h> 且自带 #pragma pack(1)。
#include <livox_lidar_def.h>

#include "sensor/lidar_types.h"

namespace ego_sensor
{

  // 把 Livox-SDK2 回调里给的一个以太网包解析成 LidarFrame。
  //
  // 用法（对应 SDK 的 LivoxLidarPointCloudCallBack）：
  //   void cb(uint32_t handle, uint8_t dev_type, LivoxLidarEthernetPacket* pkt, void* ctx) {
  //     auto* p = static_cast<MyCtx*>(ctx);
  //     p->parser.parsePacket(*pkt, p->t_epoch_offset, p->frame);   // 累积
  //     if (p->frameDone()) { p->consume(std::move(p->frame)); p->frame = {}; }
  //   }
  // “一帧”的切分由驱动决定（按 frame_cnt 或固定 100 ms 累积窗口），不在这里猜。
  class LivoxParser
  {
  public:
    struct Config
    {
      // Mid-360S：近处盲区 0.1 m；截止量程 100 m（@10% 反射率典型值 40 m）
      double min_range = 0.10;
      double max_range = 100.0;
      // 只取第一回波（Mid-360S 是单回波；双回波机型打开才取两回波）
      bool keep_second_echo = false;
    };

    void setConfig(const Config &c) { cfg_ = c; }

    // 包的原始雷达时间戳（纳秒，未做时钟换算）—— 整数比较，适合做边界/丢包判断
    static uint64_t packetTimeNs(const LivoxLidarEthernetPacket &pkt);

    // 包的基准时刻（位姿时钟，秒）= t_epoch_offset + timestamp_ns*1e-9
    static double packetTime(const LivoxLidarEthernetPacket &pkt, double t_epoch_offset);

    // 解析一个包并**追加**到 frame（调用方负责每帧清空）。
    // t_epoch_offset: 雷达时间戳 -> 位姿时钟 的偏移（秒）。
    //   该偏移由时间同步环节（PTPv2 / GPS-PPS）确定，不在本函数里猜。
    // 点时间 = packetTime + i*time_interval*1e-7/(dot_num-1)
    // 全零点（无效回波）由 min_range 过滤掉。
    // 返回 false 表示包不可用（长度不足 / 未知 data_type）。
    bool parsePacket(const LivoxLidarEthernetPacket &pkt, double t_epoch_offset, LidarFrame &frame) const;

  private:
    Config cfg_;
  };

} // namespace ego_sensor

#endif
