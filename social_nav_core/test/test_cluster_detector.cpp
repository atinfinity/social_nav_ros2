#include <cmath>
#include <vector>

#include "mini_test.hpp"
#include "social_nav_core/cluster_detector.hpp"

using namespace social_nav;

namespace {
void addCylinder(std::vector<Point3>& pts, double cx, double cy, double r) {
  for (double z : {0.5, 0.9, 1.3})
    for (int k = 0; k < 24; ++k) {
      const double a = 2 * M_PI * k / 24;
      pts.push_back({static_cast<float>(cx + r * std::cos(a)), static_cast<float>(cy + r * std::sin(a)),
                     static_cast<float>(z)});
    }
}
void addWall(std::vector<Point3>& pts, double y, double x0, double x1) {
  for (double x = x0; x <= x1; x += 0.05) pts.push_back({static_cast<float>(x), static_cast<float>(y), 1.0f});
}
std::vector<Point3> scene() {
  std::vector<Point3> pts;
  addCylinder(pts, 2.0, -1.0, 0.15);
  addCylinder(pts, 4.0, 1.0, 0.15);
  addWall(pts, 3.0, -2.0, 6.0);
  for (double x = 0; x < 3; x += 0.05) pts.push_back({static_cast<float>(x), 0.0f, 0.1f});   // 床付近(高さゲート外)
  return pts;
}
}  // namespace

TEST(two_cylinders_and_wall_give_two_detections) {
  LidarClusterDetector det(ClusterParams{});
  const auto d = det.detect(scene(), {0, 0, 0});
  EXPECT_TRUE(d.size() == 2);
  if (d.size() != 2) return;
  bool a = false, b = false;
  for (const auto& x : d) {
    if (std::hypot(x.pos.x - 2.0, x.pos.y + 1.0) < 0.03) a = true;
    if (std::hypot(x.pos.x - 4.0, x.pos.y - 1.0) < 0.03) b = true;
    EXPECT_TRUE(x.source == DetectionSource::LIDAR);
  }
  EXPECT_TRUE(a && b);
}

TEST(wall_alone_is_rejected_by_width_gate) {
  std::vector<Point3> pts;
  addWall(pts, 3.0, -2.0, 6.0);
  LidarClusterDetector det(ClusterParams{});
  EXPECT_TRUE(det.detect(pts, {0, 0, 0}).empty());
}

TEST(detections_are_transformed_to_odom) {
  std::vector<Point3> pts;
  addCylinder(pts, 2.0, 0.0, 0.15);
  LidarClusterDetector det(ClusterParams{});
  const auto d = det.detect(pts, {1.0, 2.0, M_PI / 2});   // base +x → odom +y
  EXPECT_TRUE(d.size() == 1);
  if (d.size() != 1) return;
  EXPECT_NEAR(d[0].pos.x, 1.0, 0.03);
  EXPECT_NEAR(d[0].pos.y, 4.0, 0.03);
}

TEST(range_and_min_points_gates) {
  std::vector<Point3> pts;
  addCylinder(pts, 12.0, 0.0, 0.15);   // max_range 10 の外
  LidarClusterDetector det(ClusterParams{});
  EXPECT_TRUE(det.detect(pts, {0, 0, 0}).empty());

  std::vector<Point3> few{{2.0f, 0.0f, 1.0f}, {2.1f, 0.0f, 1.0f}, {2.2f, 0.0f, 1.0f}};   // 3 点 < min_points
  EXPECT_TRUE(det.detect(few, {0, 0, 0}).empty());
}

MINI_TEST_MAIN()
