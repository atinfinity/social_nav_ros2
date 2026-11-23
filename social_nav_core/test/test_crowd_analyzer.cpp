#include <cmath>
#include <vector>

#include "mini_test.hpp"
#include "social_nav_core/crowd_analyzer.hpp"

using namespace social_nav;

namespace {
Agent person(uint32_t id, double x, double y, double vx = 0, double vy = 0) {
  Agent a{};
  a.id = id; a.pos = {x, y}; a.vel = {vx, vy};
  return a;
}
std::vector<Pose2D> straight(double len, double step = 0.5) {
  std::vector<Pose2D> p;
  for (double x = 0; x <= len + 1e-9; x += step) p.push_back({x, 0, 0});
  return p;
}
}  // namespace

TEST(crowd_density_counts_people_in_radius) {
  TrackedPeople pp;
  pp.agents = {person(1, 1, 0), person(2, 0, 1), person(3, 10, 0)};
  const double pi = 3.14159265358979323846;
  EXPECT_NEAR(crowdDensity(pp, {0, 0}, 2.0), 2.0 / (pi * 4.0), 1e-9);
  EXPECT_NEAR(crowdDensity(TrackedPeople{}, {0, 0}, 2.0), 0.0, 1e-12);
}

TEST(crowd_speed_ratio_saturates) {
  EXPECT_NEAR(speedLimitRatio(0.0, 0.1, 0.5, 0.4), 1.0, 1e-12);
  EXPECT_NEAR(speedLimitRatio(0.1, 0.1, 0.5, 0.4), 1.0, 1e-12);
  EXPECT_NEAR(speedLimitRatio(0.3, 0.1, 0.5, 0.4), 0.7, 1e-12);
  EXPECT_NEAR(speedLimitRatio(0.5, 0.1, 0.5, 0.4), 0.4, 1e-12);
  EXPECT_NEAR(speedLimitRatio(5.0, 0.1, 0.5, 0.4), 0.4, 1e-12);
}

TEST(crowd_people_on_path_sorted_and_width_gated) {
  TrackedPeople pp;
  pp.agents = {person(1, 2.0, 0.1), person(2, 1.0, -0.2), person(3, 1.5, 0.8),   // 3 は通路幅外
               person(4, 4.0, 0.0),                                               // 4 は lookahead 外
               person(5, -1.0, 0.0)};                                             // 5 は背後
  CorridorParams c;  // lookahead 2.5, half_width 0.5
  const auto r = peopleOnPath(straight(10), {0, 0, 0}, pp, c);
  EXPECT_TRUE(r.size() == 2);
  if (r.size() == 2) {
    EXPECT_TRUE(r[0].id == 2);
    EXPECT_TRUE(r[1].id == 1);
    EXPECT_NEAR(r[0].dist_along, 1.0, 1e-9);
    EXPECT_NEAR(r[1].dist_along, 2.0, 1e-9);
  }
}

TEST(crowd_speed_along_sign_on_curved_path) {
  // (0,0)→(1,0) の後 +y 方向へ曲がる経路
  std::vector<Pose2D> path{{0, 0, 0}, {1, 0, 0}, {1, 1, 1.57}, {1, 2, 1.57}};
  TrackedPeople pp;
  pp.agents = {person(1, 1.0, 1.0, 0.0, 0.6),    // 曲がった後の接線(+y)と同方向
               person(2, 1.0, 1.8, 0.0, -0.6)};  // 逆方向
  CorridorParams c; c.lookahead = 3.0;
  const auto r = peopleOnPath(path, {0, 0, 0}, pp, c);
  EXPECT_TRUE(r.size() == 2);
  if (r.size() == 2) {
    EXPECT_TRUE(r[0].id == 1);
    EXPECT_NEAR(r[0].speed_along, 0.6, 1e-9);
    EXPECT_NEAR(r[0].speed, 0.6, 1e-9);
    EXPECT_TRUE(r[1].id == 2);
    EXPECT_NEAR(r[1].speed_along, -0.6, 1e-9);
    EXPECT_NEAR(r[1].dist_along, 2.8, 1e-9);
  }
}

TEST(crowd_person_behind_robot_is_not_on_path) {
  const auto path = straight(10.0);
  TrackedPeople pp;
  pp.agents = {person(1, -0.3, 0.0), person(2, 2.0, 0.0)};   // 1: ロボット背後 0.3 m、2: 前方
  const auto r = peopleOnPath(path, {0, 0, 0}, pp, CorridorParams{});
  EXPECT_TRUE(r.size() == 1);
  if (r.size() == 1) EXPECT_TRUE(r[0].id == 2);
}

TEST(crowd_robot_before_path_start_detects_person_between) {
  // ロボットは path[0] (x=0) の 1 m 手前。人はその中間 (x=-0.5) で通路幅内
  const auto path = straight(10.0);
  TrackedPeople pp;
  pp.agents = {person(1, -0.5, 0.1)};
  const auto r = peopleOnPath(path, {-1.0, 0, 0}, pp, CorridorParams{});
  EXPECT_TRUE(r.size() == 1);
  if (r.size() == 1) {
    EXPECT_TRUE(r[0].id == 1);
    EXPECT_NEAR(r[0].dist_along, 0.5, 1e-9);   // ロボットからの距離
  }
}

TEST(crowd_robot_before_path_start_excludes_person_behind_robot) {
  const auto path = straight(10.0);
  TrackedPeople pp;
  pp.agents = {person(1, -1.5, 0.0), person(2, 1.0, 0.0)};   // 1: ロボット(x=-1)の背後
  const auto r = peopleOnPath(path, {-1.0, 0, 0}, pp, CorridorParams{});
  EXPECT_TRUE(r.size() == 1);
  if (r.size() == 1) {
    EXPECT_TRUE(r[0].id == 2);
    EXPECT_NEAR(r[0].dist_along, 2.0, 1e-9);
  }
}

MINI_TEST_MAIN()
