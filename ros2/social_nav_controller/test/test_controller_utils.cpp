#include <gtest/gtest.h>

#include <cmath>
#include <memory>
#include <random>

#include "nav2_costmap_2d/cost_values.hpp"
#include "social_nav_controller/controller_compat.hpp"
#include "social_nav_controller/controller_utils.hpp"
#include "social_nav_core/footprint.hpp"
#include "social_nav_core/motion_model.hpp"
#include "social_nav_core/sampling_controller.hpp"

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

TEST(AccelGuard, DisablesWithoutClearTime)
{
  bool disabled = false;
  EXPECT_FALSE(resolveAccelRollout(true, 0.0, disabled));
  EXPECT_TRUE(disabled);
  EXPECT_FALSE(resolveAccelRollout(true, -1.0, disabled));
  EXPECT_TRUE(disabled);
  EXPECT_TRUE(resolveAccelRollout(true, 3.0, disabled));
  EXPECT_FALSE(disabled);
  EXPECT_FALSE(resolveAccelRollout(false, 0.0, disabled));  // off は参照しない
  EXPECT_FALSE(disabled);
  EXPECT_FALSE(resolveAccelRollout(false, 3.0, disabled));
  EXPECT_FALSE(disabled);
}

TEST(AccelGuard, PeriodMismatch)
{
  EXPECT_FALSE(accelPeriodMismatch(10.0, 0.1));
  EXPECT_TRUE(accelPeriodMismatch(20.0, 0.1));
  EXPECT_TRUE(accelPeriodMismatch(0.0, 0.1));
}

// ---- door_yield の静的障害物コスト ----
namespace {
nav_msgs::msg::OccupancyGrid makeGrid(int w, int h, double res, double ox, double oy, int8_t fill)
{
  nav_msgs::msg::OccupancyGrid g;
  g.info.width = w;
  g.info.height = h;
  g.info.resolution = res;
  g.info.origin.position.x = ox;
  g.info.origin.position.y = oy;
  g.data.assign(static_cast<std::size_t>(w) * h, fill);
  return g;
}
// 幅 gap [m] の開口(y 方向)をもつ x 方向に厚み 1 セルの壁。壁は x = 40 の列、開口の中心は y = 40 セル
void setWall(nav_msgs::msg::OccupancyGrid & g, double gap_m)
{
  const int w = g.info.width, h = g.info.height;
  const double res = g.info.resolution;
  for (int y = 0; y < h; ++y) {
    const double yc = (y + 0.5) * res, c = 40.5 * res;
    if (std::abs(yc - c) >= gap_m / 2.0) {
      g.data[static_cast<std::size_t>(y) * w + 40] = 100;
    }
  }
}
double centerCost(double gap)
{
  auto g = makeGrid(81, 81, 0.05, 0.0, 0.0, 0);
  setWall(g, gap);
  const auto f = buildDistanceField(g, 65);
  return makeStaticCostFn(f, 0, 0, 0)(40.5 * 0.05, 40.5 * 0.05);
}
}  // namespace

TEST(StaticCost, FormulaMatchesSim)
{
  EXPECT_DOUBLE_EQ(staticCostFromDistance(0.34), 1.0);
  EXPECT_NEAR(staticCostFromDistance(0.35), 0.9, 1e-12);
  EXPECT_NEAR(staticCostFromDistance(0.726), 0.2, 0.001);
  EXPECT_LT(staticCostFromDistance(0.9), 0.2);
}

TEST(StaticCost, OpeningWidth)
{
  EXPECT_GE(centerCost(1.2), 0.2);
  EXPECT_LT(centerCost(2.0), 0.2);
}

TEST(DistanceField, MatchesBruteForce)
{
  std::mt19937 rng(7);
  for (int trial = 0; trial < 5; ++trial) {
    const int w = 13 + trial, h = 9 + 2 * trial;
    auto g = makeGrid(w, h, 0.1, 0.0, 0.0, 0);
    for (auto & v : g.data) {
      const int r = static_cast<int>(rng() % 10);
      v = r == 0 ? 100 : (r == 1 ? -1 : (r == 2 ? 64 : 0));  // 64 は占有でない(thresh 65)、-1 は unknown
    }
    if (trial == 4) {
      std::fill(g.data.begin(), g.data.end(), 0);  // 占有なし: 大きい値
    }
    const auto f = buildDistanceField(g, 65);
    for (int y = 0; y < h; ++y) {
      for (int x = 0; x < w; ++x) {
        double best = 1e6;
        for (int oy = 0; oy < h; ++oy) {
          for (int ox = 0; ox < w; ++ox) {
            if (g.data[static_cast<std::size_t>(oy) * w + ox] >= 65) {
              best = std::min(best, 0.1 * std::hypot(x - ox, y - oy));
            }
          }
        }
        EXPECT_NEAR(f->dist[static_cast<std::size_t>(y) * w + x], best, 1e-4);
      }
    }
  }
}

TEST(StaticCostFn, OutsideUnknownAndTransform)
{
  auto g = makeGrid(20, 20, 0.1, -1.0, -1.0, -1);  // 全 unknown
  g.data[10 * 20 + 10] = 100;                      // セル中心 (0.05, 0.05)
  const auto f = buildDistanceField(g, 65);
  // 恒等: 占有セル上は 1、unknown セルは占有セルから十分離れれば小さい、地図の外は 0
  const auto id = makeStaticCostFn(f, 0, 0, 0);
  EXPECT_DOUBLE_EQ(id(0.05, 0.05), 1.0);
  EXPECT_LT(id(-0.95, -0.95), 0.2);
  EXPECT_DOUBLE_EQ(id(5.0, 5.0), 0.0);
  EXPECT_DOUBLE_EQ(id(-1.5, 0.0), 0.0);
  // 回転 + 平行移動: odom の点 p は map の R p + t。odom の (a, b) が map の (0.05, 0.05) になるように置く
  const double yaw = 0.7, tx = 2.0, ty = -1.0;
  const double c = std::cos(yaw), s = std::sin(yaw);
  const double mx = 0.05, my = 0.05;
  // 逆変換
  const double ox = c * (mx - tx) + s * (my - ty), oy = -s * (mx - tx) + c * (my - ty);
  const auto tr = makeStaticCostFn(f, tx, ty, yaw);
  EXPECT_DOUBLE_EQ(tr(ox, oy), 1.0);
  EXPECT_LT(tr(ox + 0.9, oy), 1.0);
  // 空地図(0 x 0)は 0
  EXPECT_DOUBLE_EQ(makeStaticCostFn(buildDistanceField(nav_msgs::msg::OccupancyGrid(), 65), 0, 0, 0)(0, 0), 0.0);
}

TEST(StaticCostFn, ZeroFnIsNonEmptyAndZeroEverywhere)
{
  const auto fn = makeZeroCostFn();
  ASSERT_TRUE(static_cast<bool>(fn));
  EXPECT_DOUBLE_EQ(fn(0, 0), 0.0);
  EXPECT_DOUBLE_EQ(fn(1e3, -1e3), 0.0);
}

// static が全域 0 なら、人を含む obstacles が狭所に見えても door_yield on で発動しない(空の関数なら obstacles に戻る)
TEST(StaticCostFn, ZeroStaticNeverTriggersDoorYield)
{
  using namespace social_nav;
  ControllerParams p;
  p.door_yield = true;
  p.door_d_hold = 1.2;
  p.door_range = 6.0;
  p.door_t_gate = 3.0;
  p.door_shift = 0.0;
  auto model = std::shared_ptr<const MotionModel>(makeMotionModel("diff_drive"));
  auto fp = std::shared_ptr<const Footprint>(makeFootprint(FootprintSpec{}));
  std::vector<Pose2D> path;
  for (int i = 0; i <= 80; ++i) {path.push_back({0.1 * i, 0.0, 0.0});}
  TrackedPeople ppl;
  ppl.agents.push_back(Agent{1, {5.0, 0.6}, {0, 0}, {}, 0.3, 1.0});
  const ObstacleCostFn band = [](double x, double) {return (x >= 4.9 && x <= 5.1) ? 0.5 : 0.0;};
  {
    SamplingController c(p, model, fp);
    EXPECT_TRUE(c.compute({3.0, 0, 0}, {0.5, 0, 0}, path, ppl, band, 1.0, ObstacleCostFn{}).door_yield_active);  // 空 = 戻る
  }
  {
    SamplingController c(p, model, fp);
    const auto r = c.compute({3.0, 0, 0}, {0.5, 0, 0}, path, ppl, band, 1.0, makeZeroCostFn());
    EXPECT_FALSE(r.door_yield_active);
    EXPECT_LT(r.door_narrow_s, 0.0);
  }
}
