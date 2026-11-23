#include <cmath>

#include "mini_test.hpp"
#include "social_nav_core/proxemics.hpp"

using namespace social_nav;

namespace {
Agent walker(double vx, double vy) { return Agent{1, {0, 0}, {vx, vy}, {}, 0.3, 1.0}; }
}  // namespace

TEST(static_person_is_isotropic) {
  ProxemicsParams prm;
  Agent a = walker(0.0, 0.0);
  const double c = proxemicCost(a, {0.5, 0}, prm);
  EXPECT_NEAR(c, proxemicCost(a, {0, 0.5}, prm), 1e-12);
  EXPECT_NEAR(c, proxemicCost(a, {-0.5, 0}, prm), 1e-12);
  EXPECT_NEAR(c, proxemicCost(a, {0.5 / std::sqrt(2.0), 0.5 / std::sqrt(2.0)}, prm), 1e-12);
  EXPECT_NEAR(proxemicCost(a, {0, 0}, prm), 1.0, 1e-12);
}

TEST(walker_front_greater_than_side_greater_than_rear) {
  ProxemicsParams prm;
  Agent a = walker(1.0, 0.0);
  const double front = proxemicCost(a, {0.8, 0}, prm);
  const double side = proxemicCost(a, {0, 0.8}, prm);
  const double rear = proxemicCost(a, {-0.8, 0}, prm);
  EXPECT_TRUE(front > side);
  EXPECT_TRUE(side > rear);
  // 進行方向が回転しても同じ
  Agent b = walker(0.0, 1.0);
  EXPECT_NEAR(proxemicCost(b, {0, 0.8}, prm), front, 1e-12);
  EXPECT_NEAR(proxemicCost(b, {-0.8, 0}, prm), side, 1e-12);
  EXPECT_NEAR(proxemicCost(b, {0, -0.8}, prm), rear, 1e-12);
}

TEST(cost_in_unit_range) {
  ProxemicsParams prm;
  Agent a = walker(1.2, 0.3);
  for (double x = -5; x <= 5; x += 0.5)
    for (double y = -5; y <= 5; y += 0.5) {
      const double c = proxemicCost(a, {x, y}, prm);
      EXPECT_TRUE(c >= 0.0 && c <= 1.0);
    }
}

TEST(radius_consistent_with_cutoff) {
  ProxemicsParams prm;
  for (double v : {0.0, 0.1, 0.5, 1.3}) {
    Agent a = walker(v, 0.0);
    const double r = proxemicRadius(a, prm);
    // 最大方向(前方 / 静止なら任意)で r ちょうどの点は cutoff に一致し、外側は cutoff 未満
    EXPECT_NEAR(proxemicCost(a, {r, 0}, prm), prm.cutoff, 1e-9);
    EXPECT_TRUE(proxemicCost(a, {r * 1.01, 0}, prm) < prm.cutoff);
    // どの方向でも r の外側では cutoff 未満
    for (int i = 0; i < 72; ++i) {
      const double th = i * M_PI / 36.0;
      EXPECT_TRUE(proxemicCost(a, {1.001 * r * std::cos(th), 1.001 * r * std::sin(th)}, prm) < prm.cutoff);
    }
  }
}

MINI_TEST_MAIN()
