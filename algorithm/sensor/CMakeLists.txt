cmake_minimum_required(VERSION 3.10)
project(ego_sensor CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

find_package(Eigen3 REQUIRED)

# 只用到 Livox-SDK2 的**数据结构定义**（包/点），不需要链接 SDK
set(LIVOX_SDK2_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../lidar/Livox-SDK2"
    CACHE PATH "Livox-SDK2 root (only its headers are used)")
if(NOT EXISTS "${LIVOX_SDK2_DIR}/include/livox_lidar_def.h")
  message(FATAL_ERROR "找不到 Livox-SDK2 头文件: ${LIVOX_SDK2_DIR}/include/livox_lidar_def.h")
endif()

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${LIVOX_SDK2_DIR}/include
)

# 感知输入适配层：雷达原始包 + 时间戳位姿 -> 世界系点云
add_library(ego_sensor STATIC
  src/pose_buffer.cpp
  src/livox_parser.cpp
  src/frame_assembler.cpp
  src/scan_assembler.cpp
)

# 自检（不需要硬件，也不需要仿真）
add_executable(sensor_selftest test/sensor_selftest.cpp)
target_link_libraries(sensor_selftest ego_sensor)

option(EGO_SENSOR_BUILD_LIVOX_LIVE_TEST "Build the Livox SDK2 live point-cloud test" OFF)
if(EGO_SENSOR_BUILD_LIVOX_LIVE_TEST)
  add_subdirectory(${LIVOX_SDK2_DIR}/sdk_core ${CMAKE_CURRENT_BINARY_DIR}/livox_sdk2 EXCLUDE_FROM_ALL)

  add_executable(livox_live_test test/livox_live_test.cpp)
  target_link_libraries(livox_live_test ego_sensor livox_lidar_sdk_static pthread)
endif()
