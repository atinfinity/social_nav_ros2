#include <gtest/gtest.h>

#include <cmath>

#include "nav2_costmap_2d/cost_values.hpp"
#include "social_nav_controller/controller_compat.hpp"
#include "social_nav_controller/controller_utils.hpp"

using namespace social_nav_controller;
using social_nav::Pose2D;

TEST(PrunePath, DropsBehindAndLimitsLookahead)
{
  std::vector<Pose2D> path;
  for (int i = 0; i <= 20; ++i) {
    path.push_back({0.5 * i, 0.0, 0.0});
  }
  const auto out = prunePath(path, {2.1, 0.3, 0.0}, 3.0);
  ASSERT_FALSE(out.empty());
  EXPECT_DOUBLE_EQ(out.front().x, 2.0);
  EXPECT_DOUBLE_EQ(out.back().x, 5.0);  // 弧長 3.0 ちょうどまで含む
  EXPECT_TRUE(prunePath({}, {0, 0, 0}, 3.0).empty());
}

TEST(ObstacleCostFn, OutOfBoundsUnknownAndCost)
{
  nav2_costmap_2d::Costmap2D cm(10, 10, 0.1, 0.0, 0.0, nav2_costmap_2d::FREE_SPACE);
  cm.setCost(2, 2, 126);
  cm.setCost(5, 5, nav2_costmap_2d::NO_INFORMATION);
  const auto fn = makeObstacleCostFn(cm);
  EXPECT_DOUBLE_EQ(fn(5.0, 5.0), 1.0);             // 範囲外
  EXPECT_DOUBLE_EQ(fn(0.55, 0.55), 1.0);           // NO_INFORMATION
  EXPECT_NEAR(fn(0.25, 0.25), 126.0 / 253.0, 1e-12);
  EXPECT_DOUBLE_EQ(fn(0.85, 0.85), 0.0);
}

TEST(TransformPeople, RotatesAndTranslates)
{
  social_nav::TrackedPeople in;
  in.frame_id = "a";
  social_nav::Agent a;
  a.id = 1;
  a.pos = {1.0, 0.0};
  a.vel = {1.0, 0.0};
  a.pos_cov = {0.2, 0.0, 0.1};
  in.agents.push_back(a);
  const auto out = transformPeople(in, 1.0, 2.0, M_PI / 2, "b");
  EXPECT_EQ(out.frame_id, "b");
  EXPECT_NEAR(out.agents[0].pos.x, 1.0, 1e-12);
  EXPECT_NEAR(out.agents[0].pos.y, 3.0, 1e-12);
  EXPECT_NEAR(out.agents[0].vel.x, 0.0, 1e-12);
  EXPECT_NEAR(out.agents[0].vel.y, 1.0, 1e-12);
  EXPECT_NEAR(out.agents[0].pos_cov.xx, 0.1, 1e-12);
  EXPECT_NEAR(out.agents[0].pos_cov.yy, 0.2, 1e-12);
}

TEST(SpeedLimitRatio, Conversions)
{
  using compat::speedLimitRatio;
  EXPECT_DOUBLE_EQ(speedLimitRatio(0.0, false, 0.8), 1.0);
  EXPECT_DOUBLE_EQ(speedLimitRatio(50.0, true, 0.8), 0.5);
  EXPECT_DOUBLE_EQ(speedLimitRatio(0.4, false, 0.8), 0.5);
  EXPECT_DOUBLE_EQ(speedLimitRatio(2.0, false, 0.8), 1.0);
}
