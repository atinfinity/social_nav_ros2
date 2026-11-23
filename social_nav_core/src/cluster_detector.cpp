#include "social_nav_core/cluster_detector.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <utility>

namespace social_nav {

LidarClusterDetector::LidarClusterDetector(ClusterParams p) : p_(p) {}

std::vector<Detection> LidarClusterDetector::detect(const std::vector<Point3>& pts_base,
                                                    const Pose2D& base_in_odom) const {
  // 高さ・距離ゲートを通した点を水平グリッドのセルに振り分ける
  std::vector<Point3> pts;
  pts.reserve(pts_base.size());
  std::map<std::pair<int, int>, std::vector<std::size_t>> cells;
  const double r2max = p_.max_range * p_.max_range;
  for (const auto& q : pts_base) {
    if (q.z < p_.z_min || q.z > p_.z_max) continue;
    if (static_cast<double>(q.x) * q.x + static_cast<double>(q.y) * q.y > r2max) continue;
    const int ci = static_cast<int>(std::floor(q.x / p_.cell));
    const int cj = static_cast<int>(std::floor(q.y / p_.cell));
    cells[{ci, cj}].push_back(pts.size());
    pts.push_back(q);
  }

  const double c = std::cos(base_in_odom.theta), s = std::sin(base_in_odom.theta);
  std::vector<Detection> out;
  std::map<std::pair<int, int>, bool> visited;
  for (const auto& kv : cells) {
    if (visited[kv.first]) continue;
    // 8 近傍の連結成分
    std::vector<std::pair<int, int>> stack{kv.first};
    visited[kv.first] = true;
    std::vector<std::size_t> members;
    while (!stack.empty()) {
      const auto cur = stack.back();
      stack.pop_back();
      const auto& v = cells.at(cur);
      members.insert(members.end(), v.begin(), v.end());
      for (int di = -1; di <= 1; ++di) {
        for (int dj = -1; dj <= 1; ++dj) {
          const std::pair<int, int> nb{cur.first + di, cur.second + dj};
          if (cells.count(nb) == 0 || visited[nb]) continue;
          visited[nb] = true;
          stack.push_back(nb);
        }
      }
    }
    if (static_cast<int>(members.size()) < p_.min_points) continue;

    double sx = 0, sy = 0;
    double min_x = 1e30, max_x = -1e30, min_y = 1e30, max_y = -1e30;
    for (auto i : members) {
      sx += pts[i].x;
      sy += pts[i].y;
      min_x = std::min<double>(min_x, pts[i].x);
      max_x = std::max<double>(max_x, pts[i].x);
      min_y = std::min<double>(min_y, pts[i].y);
      max_y = std::max<double>(max_y, pts[i].y);
    }
    const double width = std::max(max_x - min_x, max_y - min_y);
    if (width < p_.min_width || width > p_.max_width) continue;

    const double n = static_cast<double>(members.size());
    const double bx = sx / n, by = sy / n;
    Detection d;
    d.pos = {base_in_odom.x + c * bx - s * by, base_in_odom.y + s * bx + c * by};
    d.cov = {0.1 * 0.1, 0.0, 0.1 * 0.1};   // 等方 σ=0.1 m(回転しても不変)
    d.radius = std::clamp(0.5 * width, 0.15, 0.5 * p_.max_width);
    d.source = DetectionSource::LIDAR;
    out.push_back(d);
  }
  return out;
}

}  // namespace social_nav
