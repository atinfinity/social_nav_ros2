#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "social_nav_perception/perception_utils.hpp"

using namespace social_nav_perception;

namespace
{
nav_msgs::msg::OccupancyGrid makeMap()
{
  nav_msgs::msg::OccupancyGrid m;
  m.info.resolution = 0.1;
  m.info.width = 100;
  m.info.height = 100;
  m.info.origin.orientation.w = 1.0;
  m.data.assign(100 * 100, 0);
  m.data[50 * 100 + 50] = 100;  // セル (50,50) = 中心 (5.05, 5.05)
  return m;
}
}  // namespace

TEST(PerceptionUtils, CloudToPointsDropsNonFinite)
{
  sensor_msgs::msg::PointCloud2 c;
  sensor_msgs::PointCloud2Modifier mod(c);
  mod.setPointCloud2FieldsByString(1, "xyz");
  mod.resize(3);
  sensor_msgs::PointCloud2Iterator<float> x(c, "x"), y(c, "y"), z(c, "z");
  const float nan = std::numeric_limits<float>::quiet_NaN();
  *x = 1; *y = 2; *z = 3; ++x; ++y; ++z;
  *x = nan; *y = 0; *z = 0; ++x; ++y; ++z;
  *x = 4; *y = 5; *z = 6;
  const auto pts = cloudToPoints(c);
  ASSERT_EQ(pts.size(), 2u);
  EXPECT_FLOAT_EQ(pts[0].x, 1.f);
  EXPECT_FLOAT_EQ(pts[1].z, 6.f);
}

TEST(PerceptionUtils, NearOccupiedWithinRadius)
{
  const auto m = makeMap();
  EXPECT_TRUE(nearOccupied(m, 5.05, 5.05, 0.2, 65));
  EXPECT_TRUE(nearOccupied(m, 5.2, 5.05, 0.2, 65));   // 0.15 m 離れ
}

TEST(PerceptionUtils, NearOccupiedFarAndOutside)
{
  const auto m = makeMap();
  EXPECT_FALSE(nearOccupied(m, 5.6, 5.05, 0.2, 65));  // 0.55 m 離れ
  EXPECT_FALSE(nearOccupied(m, -3.0, -3.0, 0.2, 65)); // 地図の外
  EXPECT_FALSE(nearOccupied(m, 5.05, 5.05, 0.2, 101)); // 閾値未満
}

TEST(PerceptionUtils, NearOccupiedHonorsMapOrigin)
{
  auto m = makeMap();
  m.info.origin.position.x = -10.0;
  m.info.origin.position.y = 2.0;
  EXPECT_TRUE(nearOccupied(m, -4.95, 7.05, 0.2, 65));
  EXPECT_FALSE(nearOccupied(m, 5.05, 5.05, 0.2, 65));
}
