// 特性テスト(記録用。合否の判定ではない): 静止ロボットに人 1 人が正面から近づく。
// 駆動 v0/tau = 2.6 m/s^2 が反発 A = 2.1 m/s^2 を上回るので、正面は接触ばねまで届くという見立ての確認。
// 現状の値(接触の有無と外形間の最小距離)を固定するだけで、この値が望ましいという意味ではない。
// 挙動を変える変更でここが赤くなったら、値の変化を確認して更新する。乱数は使わない(決定的)。
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

#include "mini_test.hpp"
#include "sim/social_force.hpp"

using namespace social_nav;
using namespace social_nav::sim;

namespace {

struct Result { double min_gap; bool contact; };

// 人は x=+4 から x=-4 へ v0=1.3 m/s で歩く(横ずれ offset)。ロボットは原点で静止。20 s、dt 0.02
Result run(double offset) {
  const SocialForceWorld world(SfmParams{}, {});
  Pedestrian p;
  p.id = 1;
  p.pos = {4.0, offset};
  p.home = p.pos;
  p.goal = {-4.0, offset};
  p.v0 = 1.3;
  p.vel = {-1.3, 0.0};
  std::vector<Pedestrian> peds{p};
  const RobotBody robot{{0.0, 0.0}, {0.0, 0.0}, 0.3};
  double min_gap = 1e9;
  for (int i = 0; i < 1000; ++i) {
    world.step(peds, &robot, 0.02);
    min_gap = std::min(min_gap, std::hypot(peds[0].pos.x, peds[0].pos.y) - 0.3 - peds[0].radius);
  }
  return {min_gap, min_gap < 0.0};
}

}  // namespace

TEST(head_on_characterization) {
  // 横ずれ, 最小距離(2026-11 の実測値。接触あり)
  const struct { double offset, min_gap; } expected[] = {{0.0, -0.0385}, {0.1, -0.0378}, {0.3, -0.0270}};
  for (const auto& e : expected) {
    const Result r = run(e.offset);
    EXPECT_TRUE(r.contact);
    EXPECT_NEAR(r.min_gap, e.min_gap, 0.002);
  }
}

MINI_TEST_MAIN()
