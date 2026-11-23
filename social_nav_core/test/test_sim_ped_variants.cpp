// applyPedVariants(家族の再配置)の単体テスト。
#include <cmath>
#include <random>

#include "mini_test.hpp"
#include "sim/ped_variants.hpp"

using namespace social_nav;
using namespace social_nav::sim;

namespace {
double d(const Vec2& a, const Vec2& b) { return std::hypot(a.x - b.x, a.y - b.y); }

// 24 人。通路の中央に等間隔で並べ、index 0 を除く全員を動かす。anchor の位置は pos0 で与える
Scenario base(Vec2 pos0) {
  Scenario s;
  s.name = "crowd";
  s.start = {0.0, 0.0, 0.0};
  s.goal = {18.0, 0.0};
  for (int i = 0; i < 24; ++i) {
    PedInit p{};
    p.pos = {4.0 + 0.8 * (i % 12) * 1.5, (i < 12) ? -2.0 : 2.0};
    p.goal = {24.0, p.pos.y};
    p.v0 = 1.2;
    p.frozen = false;
    s.peds.push_back(p);
  }
  (void)pos0;
  return s;
}

void checkConstraints(const Scenario& s, const Scenario& orig) {
  for (std::size_t i = 0; i < s.peds.size(); ++i) {
    if (s.peds[i].kind != 1) continue;
    const Vec2 q = s.peds[i].pos;
    EXPECT_TRUE(std::fabs(q.y) <= 3.5 + 1e-9);
    if (s.peds[i].pos.x != orig.peds[i].pos.x || q.y != orig.peds[i].pos.y) {   // 動かされた人
      EXPECT_TRUE(d(q, {s.start.x, s.start.y}) > 1.5);
      for (std::size_t j = 0; j < s.peds.size(); ++j) {
        if (j == i || (s.peds[j].kind == 1 && s.peds[j].group_id == s.peds[i].group_id)) continue;
        EXPECT_TRUE(d(q, s.peds[j].pos) > 0.8);
      }
    }
  }
}
}  // namespace

TEST(variants_wall_side_anchor_flips_or_skips) {
  for (uint32_t seed = 0; seed < 50; ++seed) {
    Scenario s = base({});
    // 全員を壁際(y = 3.5)に置く。perp は y 方向(進行方向 +x なので +perp = +y)で壁の外へ出る
    for (PedInit& p : s.peds) { p.pos.y = 3.5; p.goal.y = 3.5; }
    for (std::size_t i = 0; i < s.peds.size(); ++i) s.peds[i].pos.x = 4.0 + 3.0 * i;
    const Scenario orig = s;
    applyPedVariants(s, seed);
    checkConstraints(s, orig);
  }
}

TEST(variants_near_start_anchor_respects_start_distance) {
  for (uint32_t seed = 0; seed < 50; ++seed) {
    Scenario s = base({});
    for (std::size_t i = 0; i < s.peds.size(); ++i) {
      s.peds[i].pos = {2.0 + 3.0 * (i % 8), -3.0 + 2.0 * (i / 8)};   // 始点の近くにも置く
      s.peds[i].goal = {s.peds[i].pos.x, s.peds[i].pos.y + 5.0};      // perp = -x 方向
    }
    s.peds[0].pos = {1.8, 0.0};
    s.peds[0].goal = {1.8, 5.0};
    const Scenario orig = s;
    applyPedVariants(s, seed);
    checkConstraints(s, orig);
  }
}

TEST(variants_deterministic_and_non_crowd_untouched) {
  Scenario a = base({}), b = base({});
  applyPedVariants(a, 7);
  applyPedVariants(b, 7);
  for (std::size_t i = 0; i < a.peds.size(); ++i) EXPECT_TRUE(a.peds[i].pos.x == b.peds[i].pos.x && a.peds[i].kind == b.peds[i].kind);
  Scenario c = base({});
  c.name = "follow";
  applyPedVariants(c, 7);
  for (const PedInit& p : c.peds) EXPECT_TRUE(p.kind == 0);
}

MINI_TEST_MAIN()
