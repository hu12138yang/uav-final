// 感知输入适配层自检（不需要硬件、不需要仿真）：
//   1) Livox 包解析：mm->m、逐点时间戳
//   2) 逐点去畸变：运动时能否用"每点自己的采样时刻位姿"精确还原世界系点云，
//      以及"整帧共用一个位姿"的朴素做法会糊多少
#include <cstddef>
#include <cstring>
#include <cmath>
#include <cstdio>
#include <vector>

#include "sensor/frame_assembler.h"
#include "sensor/livox_parser.h"
#include "sensor/scan_assembler.h"

using namespace ego_sensor;

namespace
{
  int g_fail = 0;
  void check(bool ok, const char *what)
  {
    std::printf("  [%s] %s\n", ok ? " OK " : "FAIL", what);
    if (!ok)
      g_fail++;
  }

  void writeTs(LivoxLidarEthernetPacket &pkt, uint64_t ns_le)
  {
    for (int i = 0; i < 8; ++i)
      pkt.timestamp[i] = (uint8_t)((ns_le >> (8 * i)) & 0xFF);
  }

  // 构造一个"直角坐标高精度"包，返回裸缓冲
  std::vector<uint8_t> makePacket(const std::vector<Eigen::Vector3d> &pts_mm, uint64_t ts_ns,
                                  uint16_t time_interval, uint8_t data_type = kLivoxLidarCartesianCoordinateHighData)
  {
    const size_t n = pts_mm.size();
    std::vector<uint8_t> buf(offsetof(LivoxLidarEthernetPacket, data) + sizeof(LivoxLidarCartesianHighRawPoint) * n, 0);
    auto *pkt = reinterpret_cast<LivoxLidarEthernetPacket *>(buf.data());
    pkt->version = 5;
    pkt->length = (uint16_t)buf.size();
    pkt->time_interval = time_interval * (n > 1 ? n - 1 : 0);
    pkt->dot_num = (uint16_t)n;
    pkt->udp_cnt = 0;
    pkt->frame_cnt = 0;
    pkt->data_type = data_type;
    pkt->time_type = 0x01; // PTP
    std::memset(pkt->rsvd, 0, sizeof(pkt->rsvd));
    pkt->crc32 = 0;
    writeTs(*pkt, ts_ns);
    auto *raw = reinterpret_cast<LivoxLidarCartesianHighRawPoint *>(pkt->data);
    for (size_t i = 0; i < n; ++i)
    {
      raw[i].x = (int32_t)std::lround(pts_mm[i].x());
      raw[i].y = (int32_t)std::lround(pts_mm[i].y());
      raw[i].z = (int32_t)std::lround(pts_mm[i].z());
      raw[i].reflectivity = (uint8_t)(50 + i);
      raw[i].tag = 0x10;
    }
    return buf;
  }
} // namespace

int main()
{
  // ---------------------------------------------------------------- 1. 解析
  std::printf("== 1. Livox 包解析 ==\n");
  {
    // 单位 mm
    const std::vector<Eigen::Vector3d> mm = {{1000, 0, 0}, {2000, 1000, -500}, {200, 300, 400}};
    const uint64_t ts_ns = 1000000000ull; // 1.0 s
    const uint16_t ti = 100;              // 100 * 0.1us = 10 us/点
    auto buf = makePacket(mm, ts_ns, ti);
    const auto *pkt = reinterpret_cast<const LivoxLidarEthernetPacket *>(buf.data());

    LivoxParser parser;
    LidarFrame frame;
    bool ok = parser.parsePacket(*pkt, /*t_epoch_offset=*/0.0, frame);
    check(ok, "parsePacket 返回成功");
    check(frame.points.size() == 3, "点数 = 3");
    check(std::fabs(frame.t_base - 1.0) < 1e-9, "t_base = 1.0 s");
    // mm -> m
    check((frame.points[0].p - Eigen::Vector3d(1.0, 0.0, 0.0)).norm() < 1e-9, "点0 mm->m 正确");
    check((frame.points[1].p - Eigen::Vector3d(2.0, 1.0, -0.5)).norm() < 1e-9, "点1 mm->m 正确");
    // 逐点时间戳：第 i 点 t_rel = i * 10 us
    check(std::fabs(frame.points[0].t_rel - 0.0) < 1e-12, "点0 t_rel = 0");
    check(std::fabs(frame.points[1].t_rel - 10e-6) < 1e-12, "点1 t_rel = 10 us");
    check(std::fabs(frame.points[2].t_rel - 20e-6) < 1e-12, "点2 t_rel = 20 us");
    // 量程过滤：三点量程 1.00 / 2.29 / 0.54 m，max_range=1.5 时剩 (1,0,0) 与 (0.2,0.3,0.4)
    LivoxParser::Config c;
    c.max_range = 1.5;
    parser.setConfig(c);
    LidarFrame f2;
    parser.parsePacket(*pkt, 0.0, f2);
    check(f2.points.size() == 2, "max_range=1.5 过滤后剩 2 点");
  }

  // ------------------------------------------------- 2. 去畸变（真实运动）
  std::printf("\n== 2. 逐点去畸变（帧内运动）==\n");
  {
    // 飞机以 2 m/s 沿 +x 前进，同时以 30 deg/s 偏航
    const double v = 2.0, yaw_rate = 30.0 * M_PI / 180.0;
    const double T_frame = 0.1; // 10 Hz 激光，一帧 100 ms
    const double t_base = 10.0;

    PoseBuffer poses;
    for (int i = 0; i <= 300; ++i) // 1 kHz，覆盖 [-0.05, 0.25]
    {
      const double t = t_base - 0.05 + i * 0.001;
      TimedPose p;
      p.t = t;
      const double dt = t - t_base;
      p.p = Eigen::Vector3d(v * dt, 0.0, 0.0); // 以 t_base 时刻为原点
      p.q = Eigen::Quaterniond(Eigen::AngleAxisd(yaw_rate * dt, Eigen::Vector3d::UnitZ()));
      poses.add(p);
    }

    // 真值：世界系里的静止特征点
    const std::vector<Eigen::Vector3d> truth = {{8.0, 1.0, 0.0}, {9.0, -2.0, 1.0},
                                                {7.5, 0.5, -1.5}, {10.0, 0.0, 0.5}};

    // 模拟真实雷达：每个点在**它自己的时刻**，被当时机体系观测到
    //   p_sensor(t) = R(t)^T * (P_world - p(t))
    LidarFrame frame;
    frame.t_base = t_base;
    frame.data_type = kLivoxLidarCartesianCoordinateHighData;
    for (size_t i = 0; i < truth.size(); ++i)
    {
      const double t_rel = T_frame * (double)i / (double)(truth.size() - 1);
      TimedPose pose;
      poses.poseAt(t_base + t_rel, pose);
      const Eigen::Vector3d p_sensor = pose.q.inverse() * (truth[i] - pose.p);
      LidarPoint lp;
      lp.p = p_sensor;
      lp.t_rel = t_rel;
      lp.reflectivity = 100;
      frame.points.push_back(lp);
    }

    ScanAssembler asm_;
    std::vector<Eigen::Vector3d> deskept, naive;
    bool ok1 = asm_.toWorld(frame, poses, deskept);
    bool ok2 = asm_.toWorldNoDeskew(frame, poses, naive);
    check(ok1 && ok2, "toWorld / toWorldNoDeskew 均成功");

    double e_deskew = 0, e_naive = 0;
    for (size_t i = 0; i < truth.size(); ++i)
    {
      e_deskew = std::max(e_deskew, (deskept[i] - truth[i]).norm());
      e_naive = std::max(e_naive, (naive[i] - truth[i]).norm());
    }
    std::printf("     去畸变后最大误差 = %.4f m\n", e_deskew);
    std::printf("     朴素做法最大误差 = %.4f m   (帧内位移 %.2f m, 偏航 %.1f deg)\n",
                e_naive, v * T_frame, yaw_rate * T_frame * 180.0 / M_PI);
    check(e_deskew < 0.01, "去畸变误差 < 1 cm");
    check(e_naive > 0.1, "朴素做法误差 > 10 cm（确实会糊）");
  }

  // ------------------------------------------- 3. 静止时两条路径必须一致
  std::printf("\n== 3. 静止（对照）==\n");
  {
    PoseBuffer poses;
    const double t0 = 5.0;
    for (int i = 0; i <= 200; ++i) // 位姿覆盖 [-0.05, +0.15]，必须盖住整帧
    {
      TimedPose p;
      p.t = t0 - 0.05 + i * 0.001;
      p.p = Eigen::Vector3d(3.0, -1.0, 2.0);
      p.q = Eigen::Quaterniond::Identity();
      poses.add(p);
    }
    const std::vector<Eigen::Vector3d> truth = {{1, 0, 0}, {2, 1, 0}};
    LidarFrame frame;
    frame.t_base = t0;
    for (size_t i = 0; i < truth.size(); ++i)
    {
      LidarPoint lp;
      lp.p = truth[i] - Eigen::Vector3d(3.0, -1.0, 2.0);
      lp.t_rel = 0.05 * (double)i;
      frame.points.push_back(lp);
    }
    ScanAssembler asm_;
    std::vector<Eigen::Vector3d> a, b;
    const bool ok_a = asm_.toWorld(frame, poses, a);
    const bool ok_b = asm_.toWorldNoDeskew(frame, poses, b);
    check(ok_a && ok_b && a.size() == truth.size() && b.size() == truth.size(),
          "两条路径都成功且点数正确");
    double d = 0;
    if (ok_a && ok_b && a.size() == truth.size())
    {
      for (size_t i = 0; i < truth.size(); ++i)
      {
        d = std::max(d, (a[i] - truth[i]).norm());
        d = std::max(d, (b[i] - truth[i]).norm());
      }
    }
    else
    {
      d = 1e9;
    }
    std::printf("     两条路径最大误差 = %.6f m\n", d);
    check(d < 1e-9, "静止时两条路径都精确还原");
  }

  // ------------------------------------------------- 4. 位姿覆盖不足要报错
  std::printf("\n== 4. 位姿覆盖不足 ==\n");
  {
    PoseBuffer poses;
    TimedPose p;
    p.t = 0.0;
    poses.add(p);
    LidarFrame frame;
    frame.t_base = 100.0; // 远超位姿时间范围
    LidarPoint lp;
    lp.p = Eigen::Vector3d(1, 0, 0);
    lp.t_rel = 0.0;
    frame.points.push_back(lp);
    ScanAssembler asm_;
    std::vector<Eigen::Vector3d> out;
    check(!asm_.toWorld(frame, poses, out), "位姿不可用时返回 false（调用方应丢帧）");
  }

  // ------------------------------------- 5. 成帧（timebase + offset_time）
  std::printf("\n== 5. 成帧（包 -> 帧）==\n");
  {
    // 仿 Mid-360 的真实节奏：点时间间隔 10 us，每包 4 点 => 一包跨 40 us，
    // 包与包首尾相接（40 us 一包）；一帧 200 us = 6 个包
    const uint16_t ti = 100;   // 100 * 0.1us = 10 us
    const double pkt_span = 40e-6;
    const uint64_t ts0 = 2000000000ull; // 2.0 s
    const std::vector<Eigen::Vector3d> four = {{1000, 0, 0}, {1000, 100, 0}, {1000, 200, 0}, {1000, 300, 0}};

    FrameAssembler fa;
    FrameAssembler::Config fc;
    fc.frame_duration = 200e-6;
    fa.setConfig(fc);
    fa.setLidarId(3);

    auto send = [&](double t_abs_s) {
      auto buf = makePacket(four, (uint64_t)std::llround(t_abs_s * 1e9), ti);
      const auto *pkt = reinterpret_cast<const LivoxLidarEthernetPacket *>(buf.data());
      fa.addPacket(*pkt, 0.0);
    };

    LidarFrame f;
    send(2.0);
    check(!fa.popFrame(f), "只有 1 个包（跨度 0）时不出帧");
    for (int k = 1; k <= 4; ++k) // 到 160 us
      send(2.0 + k * pkt_span);
    check(!fa.popFrame(f), "跨度 160 us < 200 us 时不出帧");
    send(2.0 + 5 * pkt_span); // 200 us -> 满帧
    check(fa.popFrame(f), "跨度到 200 us 时出帧");
    check(f.points.size() == 24, "一帧 24 个点（6 包 × 4 点）");
    check(f.lidar_id == 3, "lidar_id 透传");
    check(std::fabs(f.t_base - 2.0) < 1e-9, "timebase = 首包时刻 2.0 s");
    if (f.points.size() == 24) // 尺寸不对就不再索引，避免越界
    {
      check(std::fabs(f.points[0].t_rel - 0.0) < 1e-12, "首包首点 offset_time = 0");
      check(std::fabs(f.points[3].t_rel - 30e-6) < 1e-12, "首包末点 = 30 us");
      check(std::fabs(f.points[4].t_rel - 40e-6) < 1e-12, "次包首点 = 40 us");
      check(std::fabs(f.points[23].t_rel - (5 * pkt_span + 30e-6)) < 1e-12, "末包末点 = 230 us");
      check(f.points[0].reflectivity == 50.0f, "reflectivity 透传");
      check(f.points[0].tag == 0x10, "tag 透传");
    }
    check(!fa.popFrame(f), "出帧后没有残留帧");

    // 丢包：间隔 1000 us >> 2 × 40 us，该包应被丢弃并计数
    const uint64_t dropped_before = fa.droppedPackets();
    send(2.0 + 5 * pkt_span + 1000e-6);
    check(fa.droppedPackets() == dropped_before + 1, "时间跳变过大 -> 丢包并计数");
  }

  std::printf("\n%s (fail=%d)\n", g_fail == 0 ? "全部通过" : "存在失败", g_fail);
  return g_fail == 0 ? 0 : 1;
}
