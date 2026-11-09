#ifndef _GRID_MAP_H
#define _GRID_MAP_H

#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

using namespace std;

// constant parameters

struct MappingParameters {

  /* map properties */
  Eigen::Vector3d map_origin_, map_size_;
  Eigen::Vector3d map_min_boundary_, map_max_boundary_;  // map range in pos
  Eigen::Vector3i map_voxel_num_;                        // map range in index
  Eigen::Vector3d local_update_range_;
  double resolution_, resolution_inv_;
  double obstacles_inflation_;
  string frame_id_;

  /* local map update and clear */
  int local_map_margin_;

  /* visualization */
  double visualization_truncate_height_, virtual_ceil_height_, ground_height_;
  /* virtual floor (minimum flight height). -inf when disabled. */
  double virtual_ground_height_;
};

// intermediate mapping data

struct MappingData {
  // inflated occupancy of each voxel, built from the sensed point cloud

  std::vector<char> occupancy_buffer_inflate_;

  // drone position

  Eigen::Vector3d camera_pos_;

  // range of updating grid

  Eigen::Vector3i local_bound_min_, local_bound_max_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class GridMap {
public:
  GridMap() {}
  ~GridMap() {}

  // occupancy map management
  void resetBuffer(Eigen::Vector3d min, Eigen::Vector3d max);

  inline void posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id);
  inline void indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos);
  inline int toAddress(const Eigen::Vector3i& id);
  inline int toAddress(int& x, int& y, int& z);
  inline bool isInMap(const Eigen::Vector3d& pos);
  inline bool isInMap(const Eigen::Vector3i& idx);

  inline int getInflateOccupancy(Eigen::Vector3d pos);

  inline void boundIndex(Eigen::Vector3i& id);

  // configure the map geometry (replaces the former ROS param loading in initMap())
  void initMap(double resolution, const Eigen::Vector3d& map_size, const Eigen::Vector3d& local_update_range,
               double obstacles_inflation, double virtual_ceil_height, double ground_height,
               int local_map_margin, const string& frame_id = "world");

  // rebuild the local inflated occupancy from one sensed point cloud (in world frame) and the current pose
  void sensedPoints(const std::vector<Eigen::Vector3d>& cloud, const Eigen::Vector3d& pose);

  // Instrumentation of the last sensedPoints() call (milliseconds). Handy when
  // tuning the map on the real machine: it shows whether the per-frame cost is
  // dominated by clearing the update box or by inflating the points.
  double prof_clear_ms = 0.0;
  double prof_inflate_ms = 0.0;

  // a virtual floor at height z (in world frame): the planner will not plan below
  // it. Set to -inf to disable. Symmetric to virtual_ceil_height.
  void setVirtualGroundHeight(double z) { mp_.virtual_ground_height_ = z; }

  // Slide the local window (x, y only) so that it is centered at `center`.
  // The occupancy buffer is shifted and cells leaving the window are dropped,
  // which turns the map into a rolling (life-long) local map.
  void recenter(const Eigen::Vector3d& center);

  inline double getResolution();

  typedef std::shared_ptr<GridMap> Ptr;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  MappingParameters mp_;
  MappingData md_;
};

/* ============================== definition of inline function
 * ============================== */

inline int GridMap::toAddress(const Eigen::Vector3i& id) {
  return id(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) + id(1) * mp_.map_voxel_num_(2) + id(2);
}

inline int GridMap::toAddress(int& x, int& y, int& z) {
  return x * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2) + y * mp_.map_voxel_num_(2) + z;
}

inline void GridMap::boundIndex(Eigen::Vector3i& id) {
  Eigen::Vector3i id1;
  id1(0) = max(min(id(0), mp_.map_voxel_num_(0) - 1), 0);
  id1(1) = max(min(id(1), mp_.map_voxel_num_(1) - 1), 0);
  id1(2) = max(min(id(2), mp_.map_voxel_num_(2) - 1), 0);
  id = id1;
}

inline int GridMap::getInflateOccupancy(Eigen::Vector3d pos) {
  if (!isInMap(pos)) return -1;

  Eigen::Vector3i id;
  posToIndex(pos, id);

  return int(md_.occupancy_buffer_inflate_[toAddress(id)]);
}

inline bool GridMap::isInMap(const Eigen::Vector3d& pos) {
  if (pos(0) < mp_.map_min_boundary_(0) + 1e-4 || pos(1) < mp_.map_min_boundary_(1) + 1e-4 ||
      pos(2) < mp_.map_min_boundary_(2) + 1e-4) {
    return false;
  }
  if (pos(0) > mp_.map_max_boundary_(0) - 1e-4 || pos(1) > mp_.map_max_boundary_(1) - 1e-4 ||
      pos(2) > mp_.map_max_boundary_(2) - 1e-4) {
    return false;
  }
  return true;
}

inline bool GridMap::isInMap(const Eigen::Vector3i& idx) {
  if (idx(0) < 0 || idx(1) < 0 || idx(2) < 0) {
    return false;
  }
  if (idx(0) > mp_.map_voxel_num_(0) - 1 || idx(1) > mp_.map_voxel_num_(1) - 1 ||
      idx(2) > mp_.map_voxel_num_(2) - 1) {
    return false;
  }
  return true;
}

inline void GridMap::posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id) {
  for (int i = 0; i < 3; ++i) id(i) = floor((pos(i) - mp_.map_origin_(i)) * mp_.resolution_inv_);
}

inline void GridMap::indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos) {
  for (int i = 0; i < 3; ++i) pos(i) = (id(i) + 0.5) * mp_.resolution_ + mp_.map_origin_(i);
}

inline double GridMap::getResolution() { return mp_.resolution_; }

#endif
