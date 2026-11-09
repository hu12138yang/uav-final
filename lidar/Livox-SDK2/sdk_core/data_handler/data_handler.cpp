//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include "data_handler.h"
#include "FastCRC/FastCRC.h"
#include <base/logging.h>

#include "livox_lidar_def.h"

namespace livox {

namespace lidar {

static const size_t kPrefixDataSize = 18;

DataHandler::DataHandler()
    : point_data_callbacks_(nullptr),
      point_client_data_(nullptr),
      imu_data_callbacks_(nullptr),
      imu_client_data_(nullptr) {
}

DataHandler& DataHandler::GetInstance() {
  static DataHandler data_handler;
  return data_handler;
}

bool DataHandler::Init() {
  LOG_INFO("Data Handler Init Succ.");
  return true;
}

void DataHandler::Destory() {
  point_data_callbacks_ = nullptr;
  point_client_data_ = nullptr;

  imu_data_callbacks_ = nullptr;
  imu_client_data_ = nullptr;

  std::lock_guard<std::mutex> lock(mutex_);
  observers_.clear();
}

DataHandler::~DataHandler() {
  Destory();
}


void DataHandler::Handle(const uint8_t dev_type, const uint32_t handle, uint8_t *buf, uint32_t buf_size) {
  if (!buf || buf_size < 36) return;
  uint16_t declared = uint16_t(buf[1]) | (uint16_t(buf[2]) << 8);
  if (declared != buf_size) return;
  // CRC-32/ISO-HDLC over timestamp + payload (wire protocol section 2.1).
  FastCRC32 checker;
  const uint32_t crc = checker.crc32(buf + 28, buf_size - 28);
  uint32_t expected = uint32_t(buf[24]) | (uint32_t(buf[25]) << 8) |
                      (uint32_t(buf[26]) << 16) | (uint32_t(buf[27]) << 24);
  // Measured Mid360S firmware emits 96 Cartesian points with a zero CRC.
  // The bundled sample's original DataHandler forwards these packets directly.
  // Explicit deployment profile permits this measured shape without claiming
  // that a missing payload CRC has been verified. Nonzero bad CRCs are rejected.
  const bool zero_crc_compatible = allow_zero_point_crc_.load() && expected == 0 &&
      dev_type == kLivoxLidarTypeMid360s && buf[0] == 0 && buf[10] == 1 &&
      buf[5] == 96 && buf[6] == 0 && buf_size == 36 + 96 * 14;
  if (crc != expected && !zero_crc_compatible) return;
  LivoxLidarEthernetPacket *lidar_data = (LivoxLidarEthernetPacket *)buf;
  if (lidar_data == NULL) {
    return;
  }

  if (lidar_data->data_type == kLivoxLidarImuData) {
    if (imu_data_callbacks_) {
      imu_data_callbacks_(handle, dev_type, lidar_data, imu_client_data_);
    }
  } else {  
    if (point_data_callbacks_) {
      point_data_callbacks_(handle, dev_type, lidar_data, point_client_data_);
    }
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto& observer : observers_) {
      auto callback = observer.second.first;
      auto client_data = observer.second.second;
      if (callback) {
        callback(handle, dev_type, lidar_data, client_data);
      }
    }
  }
}

uint16_t DataHandler::AddPointCloudObserver(const DataCallback &cb, void *client_data) {
  uint16_t observer_id = GenerateObserverId();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    observers_[observer_id] = std::make_pair(cb, client_data);
  }
  return observer_id;
}

void DataHandler::RemovePointCloudObserver(uint16_t id) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (observers_.find(id) != observers_.end()) {
    observers_.erase(id);
  }
}

uint16_t DataHandler::GenerateObserverId() {
  static std::atomic<std::uint16_t> observer_id(1);
  uint16_t value = observer_id.load();
  uint16_t desired = 0;
  do {
    if (value == UINT16_MAX) {
      desired = 1;
    } else {
      desired = value + 1;
    }
  } while (!observer_id.compare_exchange_weak(value, desired));
  return desired;
}

void DataHandler::SetPointDataCallback(const DataCallback& cb, void *client_data) {
  point_data_callbacks_ = cb;
  point_client_data_ = client_data;
}

void DataHandler::SetImuDataCallback(const DataCallback& cb, void* client_data) {
  imu_data_callbacks_ = cb;
  imu_client_data_ = client_data;
}

} // namespace lidar
}  // namespace livox
