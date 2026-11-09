cmake_minimum_required(VERSION 3.10)
project(ego_mavlink CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Eigen3 REQUIRED)

# fcu_interface.h 里用到了规划器的指令结构与感知侧位姿结构
set(CORE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../ego_planner_core)
set(SENSOR_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../sensor)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CORE_DIR}/include
  ${SENSOR_DIR}/include
)

# 纯头文件模块（编解码 + 报文 + 语义层）
add_library(ego_mavlink INTERFACE)
target_include_directories(ego_mavlink INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(mavlink_selftest test/mavlink_selftest.cpp)
target_link_libraries(mavlink_selftest ego_mavlink)
