#include <livox_lidar_api.h>
#include <livox_lidar_def.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstddef>
#include <cstdio>
#include <mutex>
#include <thread>

#include "sensor/frame_assembler.h"

namespace
{

  volatile std::sig_atomic_t g_stop = 0;
  std::atomic<uint64_t> g_packets{0};
  std::atomic<uint64_t> g_invalid_packets{0};
  std::mutex g_assembler_mutex;
  ego_sensor::FrameAssembler g_assembler;

  void stopHandler(int)
  {
    g_stop = 1;
  }

  size_t pointSize(uint8_t data_type)
  {
    switch (data_type)
    {
    case kLivoxLidarCartesianCoordinateHighData:
      return sizeof(LivoxLidarCartesianHighRawPoint);
    case kLivoxLidarCartesianCoordinateLowData:
      return sizeof(LivoxLidarCartesianLowRawPoint);
    case kLivoxLidarSphericalCoordinateData:
      return sizeof(LivoxLidarSpherPoint);
    case kLivoxLidarDoubleEchoData:
      return sizeof(LivoxLidarDoubleEchoRawPoint);
    default:
      return 0;
    }
  }

  void workModeCallback(livox_status status, uint32_t handle,
                        LivoxLidarAsyncControlResponse *response, void *)
  {
    if (status != kLivoxLidarStatusSuccess || response == nullptr || response->ret_code != 0)
    {
      std::fprintf(stderr, "[Livox] 设置 NORMAL 模式失败：status=%d handle=%u%s\n",
                   status, handle, response == nullptr ? " response=null" : "");
      return;
    }
    std::printf("[Livox] handle=%u 已设置 NORMAL 模式\n", handle);
  }

  void lidarInfoChangeCallback(uint32_t handle, const LivoxLidarInfo *info, void *)
  {
    if (info == nullptr)
    {
      std::fprintf(stderr, "[Livox] 收到空设备信息，handle=%u\n", handle);
      return;
    }

    std::printf("[Livox] 设备已发现：handle=%u SN=%s IP=%s type=%u\n",
                handle, info->sn, info->lidar_ip, static_cast<unsigned int>(info->dev_type));
    const livox_status status =
        SetLivoxLidarWorkMode(handle, kLivoxLidarNormal, workModeCallback, nullptr);
    if (status != kLivoxLidarStatusSuccess)
      std::fprintf(stderr, "[Livox] NORMAL 模式命令未能发送：status=%d handle=%u\n", status, handle);
  }

  void pointCloudCallback(uint32_t handle, uint8_t dev_type,
                          LivoxLidarEthernetPacket *packet, void *)
  {
    if (packet == nullptr)
    {
      g_invalid_packets.fetch_add(1, std::memory_order_relaxed);
      return;
    }

    const size_t point_size = pointSize(packet->data_type);
    const size_t header_size = offsetof(LivoxLidarEthernetPacket, data);
    if (point_size == 0 ||
        packet->length < header_size ||
        packet->dot_num > (packet->length - header_size) / point_size)
    {
      const uint64_t invalid = g_invalid_packets.fetch_add(1, std::memory_order_relaxed) + 1;
      if (invalid == 1 || invalid % 100 == 0)
        std::fprintf(stderr, "[数据包] 长度/类型异常：handle=%u type=%u dots=%u length=%u invalid=%llu\n",
                     handle, packet->data_type, packet->dot_num, packet->length,
                     static_cast<unsigned long long>(invalid));
      return;
    }

    g_packets.fetch_add(1, std::memory_order_relaxed);
    std::lock_guard<std::mutex> lock(g_assembler_mutex);
    g_assembler.addPacket(*packet, 0.0);

    ego_sensor::LidarFrame frame;
    while (g_assembler.popFrame(frame))
    {
      Eigen::Vector3d min_xyz = Eigen::Vector3d::Zero();
      Eigen::Vector3d max_xyz = Eigen::Vector3d::Zero();
      double min_range = 0.0;
      double max_range = 0.0;
      if (!frame.points.empty())
      {
        min_xyz = max_xyz = frame.points.front().p;
        min_range = max_range = frame.points.front().p.norm();
        for (const auto &point : frame.points)
        {
          min_xyz = min_xyz.cwiseMin(point.p);
          max_xyz = max_xyz.cwiseMax(point.p);
          min_range = std::min(min_range, point.p.norm());
          max_range = std::max(max_range, point.p.norm());
        }
      }
      std::printf("[点云帧] handle=%u dev=%u type=%u points=%zu t_base=%.9f "
                  "range=%.2f..%.2fm xyz=[%.2f..%.2f, %.2f..%.2f, %.2f..%.2f]m "
                  "packets=%llu invalid=%llu dropped=%llu",
                  handle, static_cast<unsigned int>(dev_type),
                  static_cast<unsigned int>(frame.data_type), frame.points.size(), frame.t_base,
                  min_range, max_range,
                  min_xyz.x(), max_xyz.x(), min_xyz.y(), max_xyz.y(), min_xyz.z(), max_xyz.z(),
                  static_cast<unsigned long long>(g_packets.load(std::memory_order_relaxed)),
                  static_cast<unsigned long long>(g_invalid_packets.load(std::memory_order_relaxed)),
                  static_cast<unsigned long long>(g_assembler.droppedPackets()));
      const size_t shown = frame.points.size() < 3 ? frame.points.size() : 3;
      for (size_t i = 0; i < shown; ++i)
      {
        const auto &point = frame.points[i];
        std::printf(" p%zu=(%.3f,%.3f,%.3f)m r=%.0f t=%.6f",
                    i, point.p.x(), point.p.y(), point.p.z(),
                    point.reflectivity, point.t_rel);
      }
      std::printf("\n");
      std::fflush(stdout);
    }
  }

} // namespace

int main(int argc, char **argv)
{
  if (argc != 2)
  {
    std::fprintf(stderr, "用法：%s <Livox-SDK2 配置文件>\n", argv[0]);
    return 2;
  }

  std::signal(SIGINT, stopHandler);
  std::signal(SIGTERM, stopHandler);

  if (!LivoxLidarSdkInit(argv[1]))
  {
    std::fprintf(stderr, "[致命错误] Livox SDK2 初始化失败：检查配置文件、host_ip 和网卡\n");
    LivoxLidarSdkUninit();
    return 1;
  }

  SetLivoxLidarPointCloudCallBack(pointCloudCallback, nullptr);
  SetLivoxLidarInfoChangeCallback(lidarInfoChangeCallback, nullptr);
  std::printf("[Livox] SDK 已初始化，等待设备及点云；Ctrl-C 退出\n");

  while (!g_stop)
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

  LivoxLidarSdkUninit();
  std::lock_guard<std::mutex> lock(g_assembler_mutex);
  std::printf("[结束] packets=%llu invalid=%llu frames=%llu dropped=%llu\n",
              static_cast<unsigned long long>(g_packets.load(std::memory_order_relaxed)),
              static_cast<unsigned long long>(g_invalid_packets.load(std::memory_order_relaxed)),
              static_cast<unsigned long long>(g_assembler.framesOut()),
              static_cast<unsigned long long>(g_assembler.droppedPackets()));
  return 0;
}
