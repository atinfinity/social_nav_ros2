#pragma once
#include <cmath>
#include <cstdint>
#include <vector>

#include "geometry_msgs/msg/quaternion.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "social_nav_core/cluster_detector.hpp"
#include "social_nav_core/types.hpp"

namespace social_nav_perception {

// クォータニオンの yaw(tf2 のリンクを避けるための自前実装)
inline double yawOf(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

// PointCloud2(x, y, z が float32)から有限な点だけを取り出す。
inline std::vector<social_nav::Point3> cloudToPoints(const sensor_msgs::msg::PointCloud2 & cloud)
{
  std::vector<social_nav::Point3> pts;
  pts.reserve(static_cast<size_t>(cloud.width) * cloud.height);
  sensor_msgs::PointCloud2ConstIterator<float> ix(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iy(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iz(cloud, "z");
  for (; ix != ix.end(); ++ix, ++iy, ++iz) {
    if (std::isfinite(*ix) && std::isfinite(*iy) && std::isfinite(*iz)) {
      pts.push_back({*ix, *iy, *iz});
    }
  }
  return pts;
}

// (x, y)(map 座標)から radius 以内に占有セル(値 >= occupied_threshold)があるか。
// 地図の外は「占有なし」とする。
inline bool nearOccupied(const nav_msgs::msg::OccupancyGrid & map, double x, double y, double radius,
                         int occupied_threshold)
{
  const auto & info = map.info;
  if (info.resolution <= 0.0 || info.width == 0 || info.height == 0) {
    return false;
  }
  // 地図は原点の yaw 回転を持ちうるので、地図座標系へ戻す
  const double yaw = yawOf(info.origin.orientation);
  const double dx = x - info.origin.position.x;
  const double dy = y - info.origin.position.y;
  const double lx = std::cos(yaw) * dx + std::sin(yaw) * dy;
  const double ly = -std::sin(yaw) * dx + std::cos(yaw) * dy;
  const double res = info.resolution;
  const int r_cells = static_cast<int>(std::ceil(radius / res));
  const int cx = static_cast<int>(std::floor(lx / res));
  const int cy = static_cast<int>(std::floor(ly / res));
  for (int j = cy - r_cells; j <= cy + r_cells; ++j) {
    for (int i = cx - r_cells; i <= cx + r_cells; ++i) {
      if (i < 0 || j < 0 || i >= static_cast<int>(info.width) || j >= static_cast<int>(info.height)) {
        continue;
      }
      // セル中心とクエリ点の距離で判定
      const double ex = (i + 0.5) * res - lx;
      const double ey = (j + 0.5) * res - ly;
      if (std::hypot(ex, ey) > radius + 0.5 * res) {
        continue;
      }
      if (map.data[static_cast<size_t>(j) * info.width + static_cast<size_t>(i)] >= occupied_threshold) {
        return true;
      }
    }
  }
  return false;
}

}  // namespace social_nav_perception
