#include "plan_env/grid_map.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cstring>

void GridMap::initMap(double resolution, const Eigen::Vector3d &map_size, const Eigen::Vector3d &local_update_range,
                      double obstacles_inflation, double virtual_ceil_height, double ground_height,
                      int local_map_margin, const string &frame_id)
{
  mp_.resolution_ = resolution;
  mp_.map_size_ = map_size;
  mp_.local_update_range_ = local_update_range;
  mp_.obstacles_inflation_ = obstacles_inflation;
  mp_.virtual_ceil_height_ = virtual_ceil_height;
  mp_.ground_height_ = ground_height;
  mp_.local_map_margin_ = local_map_margin;
  mp_.frame_id_ = frame_id;
  mp_.virtual_ground_height_ = -1e9; // disabled unless setVirtualGroundHeight() is called

  if (mp_.virtual_ceil_height_ - mp_.ground_height_ > mp_.map_size_(2))
  {
    mp_.virtual_ceil_height_ = mp_.ground_height_ + mp_.map_size_(2);
  }

  mp_.resolution_inv_ = 1 / mp_.resolution_;
  mp_.map_origin_ = Eigen::Vector3d(-mp_.map_size_(0) / 2.0, -mp_.map_size_(1) / 2.0, mp_.ground_height_);

  for (int i = 0; i < 3; ++i)
    mp_.map_voxel_num_(i) = ceil(mp_.map_size_(i) / mp_.resolution_);

  mp_.map_min_boundary_ = mp_.map_origin_;
  mp_.map_max_boundary_ = mp_.map_origin_ + mp_.map_size_;

  // initialize data buffers
  int buffer_size = mp_.map_voxel_num_(0) * mp_.map_voxel_num_(1) * mp_.map_voxel_num_(2);
  md_.occupancy_buffer_inflate_ = vector<char>(buffer_size, 0);
}

void GridMap::resetBuffer(Eigen::Vector3d min_pos, Eigen::Vector3d max_pos)
{

  Eigen::Vector3i min_id, max_id;
  posToIndex(min_pos, min_id);
  posToIndex(max_pos, max_id);

  boundIndex(min_id);
  boundIndex(max_id);

  /* reset occ and dist buffer */
  for (int x = min_id(0); x <= max_id(0); ++x)
    for (int y = min_id(1); y <= max_id(1); ++y)
      for (int z = min_id(2); z <= max_id(2); ++z)
      {
        md_.occupancy_buffer_inflate_[toAddress(x, y, z)] = 0;
      }
}

void GridMap::sensedPoints(const std::vector<Eigen::Vector3d> &cloud, const Eigen::Vector3d &pose)
{
  md_.camera_pos_ = pose;

  if (cloud.empty())
    return;

  if (std::isnan(md_.camera_pos_(0)) || std::isnan(md_.camera_pos_(1)) || std::isnan(md_.camera_pos_(2)))
    return;

  auto t_start = std::chrono::steady_clock::now();

  // No per-frame clearing: the map is insert-only (0 -> 1), which is monotonic
  // and therefore strictly safer for a static environment.
  //
  // The upstream code called resetBuffer(camera +- local_update_range) and then
  // re-marked only the points of the current frame. That assumes the sensor
  // covers the whole update box. It does not with a Mid-360S (vertical FOV
  // -7 .. +52 deg): a thin obstacle right below the flight line (e.g. a cable)
  // leaves the FOV before the vehicle passes over it, so the box was wiped and
  // the obstacle was forgotten -> collision.
  //
  // Forgetting now happens only when a cell leaves the rolling window
  // (see recenter()). Call resetBuffer() explicitly when the map must be
  // cleared (e.g. a new mission).
  auto t_cleared = std::chrono::steady_clock::now();

  Eigen::Vector3d p3d;
  std::vector<std::array<int, 6>> inflation_boxes;
  inflation_boxes.reserve(cloud.size());

  int inf_step = ceil(mp_.obstacles_inflation_ / mp_.resolution_);
  int inf_step_z = inf_step; // isotropic inflation (was 1: too thin for slanted / spherical surfaces)

  double max_x, max_y, max_z, min_x, min_y, min_z;

  min_x = mp_.map_max_boundary_(0);
  min_y = mp_.map_max_boundary_(1);
  min_z = mp_.map_max_boundary_(2);

  max_x = mp_.map_min_boundary_(0);
  max_y = mp_.map_min_boundary_(1);
  max_z = mp_.map_min_boundary_(2);

  for (size_t i = 0; i < cloud.size(); ++i)
  {
    p3d = cloud[i];

    /* point inside update range */
    Eigen::Vector3d devi = p3d - md_.camera_pos_;

    if (fabs(devi(0)) < mp_.local_update_range_(0) && fabs(devi(1)) < mp_.local_update_range_(1) &&
        fabs(devi(2)) < mp_.local_update_range_(2))
    {

      // Compute endpoint indices using the same world-coordinate rounding as
      // the original kernel. Filling the inclusive box preserves every voxel
      // previously touched, including points close to a floating-point boundary.
      const Eigen::Vector3d padding(inf_step * mp_.resolution_, inf_step * mp_.resolution_,
                                    inf_step_z * mp_.resolution_);
      const Eigen::Vector3d low = p3d - padding, high = p3d + padding;
      min_x = min(min_x, low.x()); min_y = min(min_y, low.y()); min_z = min(min_z, low.z());
      max_x = max(max_x, high.x()); max_y = max(max_y, high.y()); max_z = max(max_z, high.z());
      Eigen::Vector3i lower, upper;
      posToIndex(low, lower); posToIndex(high, upper);
      lower = lower.cwiseMax(Eigen::Vector3i::Zero());
      upper = upper.cwiseMin(mp_.map_voxel_num_ - Eigen::Vector3i::Ones());
      if ((lower.array() > upper.array()).any()) continue;
      inflation_boxes.push_back({lower.x(), lower.y(), lower.z(), upper.x(), upper.y(), upper.z()});
    }
  }

  // A 2.75 m inflation has 23^3 cells per raw point. Many returns cover the
  // same voxel box; process each distinct box once, then fill contiguous Z rows.
  std::sort(inflation_boxes.begin(), inflation_boxes.end());
  inflation_boxes.erase(std::unique(inflation_boxes.begin(), inflation_boxes.end()), inflation_boxes.end());
  for (const auto &box : inflation_boxes)
    for (int x = box[0]; x <= box[3]; ++x)
      for (int y = box[1]; y <= box[4]; ++y)
      {
        const int first = toAddress(Eigen::Vector3i(x, y, box[2]));
        std::fill(md_.occupancy_buffer_inflate_.begin() + first,
                  md_.occupancy_buffer_inflate_.begin() + first + box[5] - box[2] + 1, 1);
      }

  min_x = min(min_x, md_.camera_pos_(0));
  min_y = min(min_y, md_.camera_pos_(1));
  min_z = min(min_z, md_.camera_pos_(2));

  max_x = max(max_x, md_.camera_pos_(0));
  max_y = max(max_y, md_.camera_pos_(1));
  max_z = max(max_z, md_.camera_pos_(2));

  max_z = max(max_z, mp_.ground_height_);

  posToIndex(Eigen::Vector3d(max_x, max_y, max_z), md_.local_bound_max_);
  posToIndex(Eigen::Vector3d(min_x, min_y, min_z), md_.local_bound_min_);

  boundIndex(md_.local_bound_min_);
  boundIndex(md_.local_bound_max_);

  // add virtual ceiling to limit flight height (as a slab, so it is as
  // "thick" as the obstacle inflation and the planner really respects it)
  if (mp_.virtual_ceil_height_ > -0.5)
  {
    int ceil_id = floor((mp_.virtual_ceil_height_ - mp_.map_origin_(2)) * mp_.resolution_inv_) - 1;
    for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
      for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
        for (int z = ceil_id - inf_step; z <= ceil_id; ++z)
        {
          if (z < 0 || z >= mp_.map_voxel_num_(2))
            continue;
          md_.occupancy_buffer_inflate_[toAddress(x, y, z)] = 1;
        }
  }

  // add virtual ground (minimum flight height), symmetric to the virtual ceiling
  if (mp_.virtual_ground_height_ > -1e8)
  {
    int ground_id = floor((mp_.virtual_ground_height_ - mp_.map_origin_(2)) * mp_.resolution_inv_);
    for (int x = md_.local_bound_min_(0); x <= md_.local_bound_max_(0); ++x)
      for (int y = md_.local_bound_min_(1); y <= md_.local_bound_max_(1); ++y)
        for (int z = ground_id; z <= ground_id + inf_step; ++z)
        {
          if (z < 0 || z >= mp_.map_voxel_num_(2))
            continue;
          md_.occupancy_buffer_inflate_[toAddress(x, y, z)] = 1;
        }
  }

  prof_clear_ms = std::chrono::duration<double, std::milli>(t_cleared - t_start).count();
  prof_inflate_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_cleared).count();
}

void GridMap::recenter(const Eigen::Vector3d &center)
{
  const Eigen::Vector3d cur = mp_.map_origin_ + mp_.map_size_ * 0.5;
  const int sx = (int)std::lround((center.x() - cur.x()) * mp_.resolution_inv_);
  const int sy = (int)std::lround((center.y() - cur.y()) * mp_.resolution_inv_);
  if (sx == 0 && sy == 0)
    return;

  const int nx = mp_.map_voxel_num_(0);
  const int ny = mp_.map_voxel_num_(1);
  const int nz = mp_.map_voxel_num_(2);
  char *buf = md_.occupancy_buffer_inflate_.data();

  // shift along x: a slab at fixed x is contiguous (ny * nz bytes)
  if (sx != 0)
  {
    const size_t slab = (size_t)ny * nz;
    if (sx > 0) // window moves towards +x: data moves to lower indices
    {
      for (int x = 0; x < nx; ++x)
      {
        char *dst = buf + (size_t)x * slab;
        const int src = x + sx;
        if (src >= nx)
          std::memset(dst, 0, slab);
        else
          std::memmove(dst, buf + (size_t)src * slab, slab);
      }
    }
    else // iterate backwards so that the source has not been overwritten yet
    {
      for (int x = nx - 1; x >= 0; --x)
      {
        char *dst = buf + (size_t)x * slab;
        const int src = x + sx;
        if (src < 0)
          std::memset(dst, 0, slab);
        else
          std::memmove(dst, buf + (size_t)src * slab, slab);
      }
    }
  }

  // shift along y: rows of nz bytes inside each x slab
  if (sy != 0)
  {
    for (int x = 0; x < nx; ++x)
    {
      char *slab = buf + (size_t)x * ny * nz;
      if (sy > 0)
      {
        for (int y = 0; y < ny; ++y)
        {
          char *dst = slab + (size_t)y * nz;
          const int src = y + sy;
          if (src >= ny)
            std::memset(dst, 0, nz);
          else
            std::memmove(dst, slab + (size_t)src * nz, nz);
        }
      }
      else
      {
        for (int y = ny - 1; y >= 0; --y)
        {
          char *dst = slab + (size_t)y * nz;
          const int src = y + sy;
          if (src < 0)
            std::memset(dst, 0, nz);
          else
            std::memmove(dst, slab + (size_t)src * nz, nz);
        }
      }
    }
  }

  mp_.map_origin_ += Eigen::Vector3d(sx * mp_.resolution_, sy * mp_.resolution_, 0.0);
  mp_.map_min_boundary_ = mp_.map_origin_;
  mp_.map_max_boundary_ = mp_.map_origin_ + mp_.map_size_;
}
