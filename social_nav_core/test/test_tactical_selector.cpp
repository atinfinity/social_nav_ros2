#include <vector>

#include "mini_test.hpp"
#include "social_nav_core/tactical_selector.hpp"

using namespace social_nav;

namespace {
Agent person(uint32_t id, double x, double y, double vx = 0, double vy = 0) {
  Agent a{};
  a.id = id; a.pos = {x, y}; a.vel = {vx, vy};
  return a;
}
std::vector<Pose2D> straight() {
  std::vector<Pose2D> p;
  for (double x = 0; x <= 10 + 1e-9; x += 0.5) p.push_back({x, 0, 0});
  return p;
}
TrackedPeople one(double x, double y, double vx = 0, double vy = 0) {
  TrackedPeople pp;
  pp.agents.push_back(person(7, x, y, vx, vy));
  return pp;
}
// 経路から離れた位置(|y| >= 1)の人を n 人。ロボット半径 3 m 内に収まる
TrackedPeople crowd(int n, uint32_t first_id = 100) {
  TrackedPeople pp;
  const double ys[] = {1.0, -1.0, 1.5, -1.5, 2.0, -2.0};
  int k = 0;
  for (double y : ys)
    for (double x = -2; x <= 2.0 + 1e-9 && k < n; x += 1.0, ++k) pp.agents.push_back(person(first_id + k, x, y));
  return pp;
}
TacticalDecision stepAt(TacticalSelector& s, double t, const std::vector<Pose2D>& path, const TrackedPeople* pp) {
  return s.step({t, {0, 0, 0}, &path, pp});
}
}  // namespace

TEST(tactical_no_people_proceeds) {
  TacticalSelector s{TacticalParams{}};
  const auto path = straight();
  auto d = stepAt(s, 0.0, path, nullptr);
  EXPECT_TRUE(d.behavior == Behavior::PROCEED);
  EXPECT_NEAR(d.speed_ratio, 1.0, 1e-12);
  TrackedPeople empty;
  d = stepAt(s, 1.0, path, &empty);
  EXPECT_TRUE(d.behavior == Behavior::PROCEED);
  EXPECT_NEAR(d.speed_ratio, 1.0, 1e-12);
}

TEST(tactical_yield_limit_cooldown_reyield) {
  TacticalSelector s{TacticalParams{}};  // max_yield_time 5, cooldown 10
  const auto path = straight();
  const auto pp = one(1.5, 0.0);
  auto d = stepAt(s, 0.0, path, &pp);
  EXPECT_TRUE(d.behavior == Behavior::YIELD);
  EXPECT_NEAR(d.speed_ratio, 0.0, 1e-12);
  EXPECT_TRUE(d.target_id.has_value() && *d.target_id == 7);
  EXPECT_TRUE(stepAt(s, 4.9, path, &pp).behavior == Behavior::YIELD);
  EXPECT_TRUE(stepAt(s, 5.0, path, &pp).behavior == Behavior::PROCEED);
  EXPECT_TRUE(stepAt(s, 10.0, path, &pp).behavior == Behavior::PROCEED);
  EXPECT_TRUE(stepAt(s, 14.9, path, &pp).behavior == Behavior::PROCEED);
  EXPECT_TRUE(stepAt(s, 15.0, path, &pp).behavior == Behavior::YIELD);
}

TEST(tactical_follow_and_speed_ratio) {
  TacticalSelector s{TacticalParams{}};
  const auto path = straight();
  const auto far = one(2.0, 0.0, 0.6, 0.0);
  auto d = stepAt(s, 0.0, path, &far);
  EXPECT_TRUE(d.behavior == Behavior::FOLLOW);
  EXPECT_NEAR(d.speed_ratio, 0.6 / 0.8, 1e-9);
  EXPECT_TRUE(d.target_id.has_value() && *d.target_id == 7);

  TacticalSelector s2{TacticalParams{}};
  const auto close = one(1.0, 0.0, 0.6, 0.0);   // follow_gap(1.5) 未満: 比例して下げる
  d = stepAt(s2, 0.0, path, &close);
  EXPECT_TRUE(d.behavior == Behavior::FOLLOW);
  EXPECT_NEAR(d.speed_ratio, 0.75 * (1.0 / 1.5), 1e-9);
}

TEST(tactical_oncoming_is_not_follow) {
  TacticalSelector s{TacticalParams{}};
  const auto path = straight();
  const auto pp = one(2.0, 0.0, -0.6, 0.0);
  const auto d = stepAt(s, 0.0, path, &pp);
  EXPECT_TRUE(d.behavior != Behavior::FOLLOW);
  EXPECT_TRUE(d.behavior == Behavior::PROCEED);
}

TEST(tactical_wait_hysteresis_and_limit) {
  const auto path = straight();
  {
    TacticalSelector s{TacticalParams{}};
    const auto dense = crowd(30);          // 30 / (pi 9) = 1.06 > 0.8
    EXPECT_TRUE(stepAt(s, 0.0, path, &dense).behavior == Behavior::WAIT);
    const auto mid = crowd(20);            // 0.71: resume(0.6) と wait(0.8) の間 → 継続
    auto d = stepAt(s, 2.0, path, &mid);
    EXPECT_TRUE(d.behavior == Behavior::WAIT);
    EXPECT_NEAR(d.speed_ratio, 0.0, 1e-12);
    const auto sparse = crowd(10);         // 0.35 < resume → 解除
    EXPECT_TRUE(stepAt(s, 4.0, path, &sparse).behavior != Behavior::WAIT);
  }
  {
    TacticalSelector s{TacticalParams{}};  // max_wait_time 15, cooldown 10
    const auto dense = crowd(30);
    EXPECT_TRUE(stepAt(s, 0.0, path, &dense).behavior == Behavior::WAIT);
    EXPECT_TRUE(stepAt(s, 14.9, path, &dense).behavior == Behavior::WAIT);
    auto d = stepAt(s, 15.0, path, &dense);
    EXPECT_TRUE(d.behavior == Behavior::PROCEED);
    EXPECT_NEAR(d.speed_ratio, 0.4, 1e-12);
    d = stepAt(s, 24.9, path, &dense);
    EXPECT_TRUE(d.behavior == Behavior::PROCEED);
    EXPECT_NEAR(d.speed_ratio, 0.4, 1e-12);
    EXPECT_TRUE(stepAt(s, 25.0, path, &dense).behavior == Behavior::WAIT);
  }
}

TEST(tactical_empty_frame_keeps_yield_limit) {
  TacticalSelector s{TacticalParams{}};   // max_yield_time 5, min_dwell 1
  const auto path = straight();
  const auto pp = one(1.5, 0.0);
  const TrackedPeople empty;
  EXPECT_TRUE(stepAt(s, 0.0, path, &pp).behavior == Behavior::YIELD);
  EXPECT_TRUE(stepAt(s, 0.5, path, &empty).behavior == Behavior::YIELD);   // min_dwell 内の空フレーム
  EXPECT_TRUE(stepAt(s, 0.7, path, nullptr).behavior == Behavior::YIELD);
  EXPECT_TRUE(stepAt(s, 1.0, path, &pp).behavior == Behavior::YIELD);
  EXPECT_TRUE(stepAt(s, 4.9, path, &pp).behavior == Behavior::YIELD);
  EXPECT_TRUE(stepAt(s, 5.0, path, &pp).behavior == Behavior::PROCEED);    // since_=0 のまま上限
  EXPECT_TRUE(stepAt(s, 14.9, path, &pp).behavior == Behavior::PROCEED);   // cooldown
  EXPECT_TRUE(stepAt(s, 15.0, path, &pp).behavior == Behavior::YIELD);
}

TEST(tactical_empty_frame_keeps_wait_limit_and_cooldown) {
  const auto path = straight();
  const TrackedPeople empty;
  const auto dense = crowd(30);
  {
    TacticalSelector s{TacticalParams{}};   // max_wait_time 15
    EXPECT_TRUE(stepAt(s, 0.0, path, &dense).behavior == Behavior::WAIT);
    EXPECT_TRUE(stepAt(s, 0.5, path, &empty).behavior == Behavior::WAIT);    // min_dwell 内
    EXPECT_TRUE(stepAt(s, 1.0, path, &dense).behavior == Behavior::WAIT);
    EXPECT_TRUE(stepAt(s, 14.9, path, &dense).behavior == Behavior::WAIT);
    EXPECT_TRUE(stepAt(s, 15.0, path, &dense).behavior == Behavior::PROCEED);
  }
  {
    TacticalSelector s{TacticalParams{}};   // 上限到達フレームが空でも cooldown が立つ
    EXPECT_TRUE(stepAt(s, 0.0, path, &dense).behavior == Behavior::WAIT);
    auto d = stepAt(s, 15.0, path, &empty);
    EXPECT_TRUE(d.behavior == Behavior::PROCEED);
    EXPECT_NEAR(d.speed_ratio, 0.4, 1e-12);                                  // cooldown 中は空でも min_ratio
    EXPECT_TRUE(stepAt(s, 20.0, path, &dense).behavior == Behavior::PROCEED);
    EXPECT_TRUE(stepAt(s, 25.0, path, &dense).behavior == Behavior::WAIT);
  }
  {
    TacticalSelector s{TacticalParams{}};   // cooldown 中の空フレームを挟んでも 0.4、終了後の空は 1.0
    EXPECT_TRUE(stepAt(s, 0.0, path, &dense).behavior == Behavior::WAIT);
    EXPECT_TRUE(stepAt(s, 15.0, path, &dense).behavior == Behavior::PROCEED);
    EXPECT_NEAR(stepAt(s, 20.0, path, &dense).speed_ratio, 0.4, 1e-12);
    EXPECT_NEAR(stepAt(s, 21.0, path, &empty).speed_ratio, 0.4, 1e-12);
    EXPECT_NEAR(stepAt(s, 22.0, path, &dense).speed_ratio, 0.4, 1e-12);
    auto d = stepAt(s, 25.0, path, &empty);
    EXPECT_TRUE(d.behavior == Behavior::PROCEED);
    EXPECT_NEAR(d.speed_ratio, 1.0, 1e-12);
  }
}

TEST(tactical_cooldown_ratio_is_min_ratio_not_density_ratio) {
  const auto path = straight();
  const auto dense = crowd(30);
  const auto light = crowd(10);   // 密度 0.354: 密度由来の比は 0.4 より明確に大きい
  const double dens_ratio = speedLimitRatio(crowdDensity(light, {0, 0}, 3.0), 0.1, 0.5, 0.4);
  EXPECT_TRUE(dens_ratio > 0.55);
  TacticalSelector s{TacticalParams{}};
  EXPECT_TRUE(stepAt(s, 0.0, path, &dense).behavior == Behavior::WAIT);
  EXPECT_TRUE(stepAt(s, 15.0, path, &dense).behavior == Behavior::PROCEED);
  auto d = stepAt(s, 20.0, path, &light);                  // cooldown 中: min_ratio
  EXPECT_TRUE(d.behavior == Behavior::PROCEED);
  EXPECT_NEAR(d.speed_ratio, 0.4, 1e-12);
  d = stepAt(s, 25.0, path, &light);                       // cooldown 明け: 密度由来
  EXPECT_TRUE(d.behavior == Behavior::PROCEED);
  EXPECT_NEAR(d.speed_ratio, dens_ratio, 1e-12);
}

TEST(tactical_wait_has_priority_over_yield) {
  TacticalSelector s{TacticalParams{}};
  const auto path = straight();
  auto pp = crowd(30);
  pp.agents.push_back(person(7, 1.5, 0.0));   // 経路上の静止者
  const auto d = stepAt(s, 0.0, path, &pp);
  EXPECT_TRUE(d.behavior == Behavior::WAIT);
  EXPECT_NEAR(d.speed_ratio, 0.0, 1e-12);
}

TEST(tactical_min_dwell_prevents_flapping) {
  TacticalParams prm;
  prm.min_dwell = 2.0;
  TacticalSelector s{prm};
  const auto path = straight();
  const auto blocker = one(1.5, 0.0);
  const auto gone = one(1.5, 3.0);   // 通路外へ
  EXPECT_TRUE(stepAt(s, 0.0, path, &blocker).behavior == Behavior::YIELD);
  EXPECT_TRUE(stepAt(s, 0.5, path, &gone).behavior == Behavior::YIELD);
  EXPECT_TRUE(stepAt(s, 1.9, path, &gone).behavior == Behavior::YIELD);
  EXPECT_TRUE(stepAt(s, 2.0, path, &gone).behavior == Behavior::PROCEED);
}

TEST(tactical_reset_clears_state) {
  TacticalSelector s{TacticalParams{}};
  const auto path = straight();
  const auto pp = one(1.5, 0.0);
  EXPECT_TRUE(stepAt(s, 0.0, path, &pp).behavior == Behavior::YIELD);
  EXPECT_TRUE(stepAt(s, 5.0, path, &pp).behavior == Behavior::PROCEED);   // cooldown 中
  s.reset();
  const auto d = stepAt(s, 6.0, path, &pp);                                // cooldown 解除済み
  EXPECT_TRUE(d.behavior == Behavior::YIELD);
  EXPECT_TRUE(d.target_id.has_value());
}

MINI_TEST_MAIN()
