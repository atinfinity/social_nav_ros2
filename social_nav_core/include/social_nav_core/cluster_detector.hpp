#pragma once
#include <vector>

#include "social_nav_core/types.hpp"

namespace social_nav {

enum class DetectionSource { LIDAR, RGBD };
struct Detection { Vec2 pos; Cov2 cov; double radius; DetectionSource source; };

struct ClusterParams { double z_min{0.3}, z_max{1.5}, cell{0.1}, min_width{0.2}, max_width{0.8},
                       max_range{10.0}; int min_points{5}; };

// 3D 点群(base 座標)から人らしい塊を検出する。出力位置は odom 座標。
class LidarClusterDetector {
 public:
  explicit LidarClusterDetector(ClusterParams p);
  std::vector<Detection> detect(const std::vector<Point3>& pts_base, const Pose2D& base_in_odom) const;

 private:
  ClusterParams p_;
};

}  // namespace social_nav
