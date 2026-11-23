#include <cmath>
#include <memory>
#include <stdexcept>
#include <vector>

#include "mini_test.hpp"
#include "social_nav_core/trajectory_cost.hpp"

using namespace social_nav;

namespace {
constexpr std::size_t kN = 20;
constexpr double kDt = 0.1;

std::shared_ptr<const Footprint> circle() { return std::make_shared<CircleFootprint>(0.3); }

TrackedPeople onePerson(Vec2 pos, Vec2 vel) {
  TrackedPeople p;
  p.agents.push_back(Agent{1, pos, vel, {}, 0.3, 1.0});
  return p;
}

struct Traj {
  std::vector<float> x, y, yaw;
  TrajectoryView view() const { return {x.data(), y.data(), yaw.data(), x.size(), 1}; }
};

Traj line(double x0, double y0, double vx, double vy) {
  Traj t;
  for (std::size_t k = 0; k < kN; ++k) {
    t.x.push_back(static_cast<float>(x0 + vx * kDt * k));
    t.y.push_back(static_cast<float>(y0 + vy * kDt * k));
    t.yaw.push_back(static_cast<float>(std::atan2(vy, vx)));
  }
  return t;
}

double scoreOf(const TrackedPeople& people, const Traj& t) {
  SocialTrajectoryCost c(SocialCostParams{}, circle());
  c.prepare(people, {0, 0}, 0.0, kDt, kN);
  return c.score(t.view());
}
}  // namespace

TEST(far_trajectory_cheaper_than_near) {
  const auto people = onePerson({2, 0}, {0, 0});
  // 軌道が人(x=2)の横を通過するよう速度 1.5 m/s(2 s で x=0→2.85)。0.8 m の横ずれと 1.0 m の横ずれを比べる
  const double near = scoreOf(people, line(0, 0.8, 1.5, 0));
  const double far = scoreOf(people, line(0, 1.0, 1.5, 0));
  EXPECT_TRUE(near > 0.0);
  EXPECT_TRUE(far < near);
}

TEST(crossing_in_front_costs_more_than_behind) {
  // 人は +x 方向へ歩く。ロボットは y 方向に横切る(正面側 x=2.4 / 背後側 x=1.6、人の初期位置 x=2)
  const auto people = onePerson({2, 0}, {1.0, 0});
  const double front = scoreOf(people, line(2.8, -1.0, 0, 1.0));
  const double behind = scoreOf(people, line(1.2, -1.0, 0, 1.0));
  EXPECT_TRUE(front > behind);
}

TEST(stride_one_equals_stride_n) {
  const auto people = onePerson({1.5, 0.3}, {-0.5, 0});
  const Traj t = line(0, 0, 0.6, 0.1);
  SocialTrajectoryCost c(SocialCostParams{}, circle());
  c.prepare(people, {0, 0}, 0.0, kDt, kN);
  const double s1 = c.score(t.view());
  // x, y, yaw を 3 本交互に並べた 1 本の配列(stride 3)
  std::vector<float> xs(kN * 3), ys(kN * 3), yaws(kN * 3);
  for (std::size_t k = 0; k < kN; ++k) {
    xs[k * 3] = t.x[k]; ys[k * 3] = t.y[k]; yaws[k * 3] = t.yaw[k];
  }
  const TrajectoryView v{xs.data(), ys.data(), yaws.data(), kN, 3};
  EXPECT_TRUE(s1 > 0.0);
  EXPECT_NEAR(c.score(v), s1, 1e-12);
}

TEST(yaw_nullptr_equals_circle_yaw) {
  const auto people = onePerson({1.0, 0.2}, {0, 0});
  const Traj t = line(0, 0, 0.5, 0.2);
  SocialTrajectoryCost c(SocialCostParams{}, circle());
  c.prepare(people, {0, 0}, 0.0, kDt, kN);
  const TrajectoryView v{t.x.data(), t.y.data(), nullptr, kN, 1};
  EXPECT_TRUE(c.score(t.view()) > 0.0);
  EXPECT_NEAR(c.score(v), c.score(t.view()), 1e-12);
}

TEST(no_people_gives_zero) {
  SocialTrajectoryCost c(SocialCostParams{}, circle());
  c.prepare(TrackedPeople{}, {0, 0}, 0.0, kDt, kN);
  EXPECT_TRUE(c.empty());
  EXPECT_NEAR(c.score(line(0, 0, 0.5, 0).view()), 0.0, 0.0);
  // max_range 外・低 confidence の人も除外されて empty
  TrackedPeople p = onePerson({20, 0}, {0, 0});
  c.prepare(p, {0, 0}, 0.0, kDt, kN);
  EXPECT_TRUE(c.empty());
  p = onePerson({1, 0}, {0, 0});
  p.agents[0].confidence = 0.1;
  c.prepare(p, {0, 0}, 0.0, kDt, kN);
  EXPECT_TRUE(c.empty());
}

TEST(collision_adds_collision_cost) {
  const auto people = onePerson({0.5, 0}, {0, 0});
  Traj t = line(0, 0, 0, 0);  // その場。距離 0.5 < 0.3 + 0.3 + 0.1
  SocialCostParams prm;
  SocialTrajectoryCost c(prm, circle());
  c.prepare(people, {0, 0}, 0.0, kDt, kN);
  EXPECT_TRUE(c.score(t.view()) > prm.collision_cost * 0.9);
}

TEST(null_footprint_throws) {
  const SocialCostParams prm{};
  EXPECT_THROW((SocialTrajectoryCost{prm, nullptr}), std::invalid_argument);
}

MINI_TEST_MAIN()
