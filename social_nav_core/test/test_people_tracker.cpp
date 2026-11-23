#include <algorithm>
#include <cmath>
#include <limits>
#include <random>
#include <set>
#include <vector>

#include "mini_test.hpp"
#include "social_nav_core/people_tracker.hpp"

using namespace social_nav;

namespace {
Detection makeDet(double x, double y, DetectionSource src = DetectionSource::LIDAR) {
  return Detection{{x, y}, {0.05 * 0.05, 0.0, 0.05 * 0.05}, 0.2, src};
}
}  // namespace

TEST(velocity_error_rms_below_015_for_constant_velocity_walker) {
  std::mt19937 rng(12345);
  std::normal_distribution<double> noise(0.0, 0.05);
  PeopleTracker tr(TrackerParams{});
  const double vx = 1.0, vy = 0.3, dt = 0.1;
  double sum = 0;
  int n = 0;
  for (int k = 0; k < 150; ++k) {
    const double t = k * dt;
    tr.update({makeDet(vx * t + noise(rng), vy * t + noise(rng))}, t);
    if (k < 30) continue;   // 収束待ち
    const auto p = tr.confirmed("odom");
    if (p.agents.size() != 1) { EXPECT_TRUE(false); continue; }
    const double ex = p.agents[0].vel.x - vx, ey = p.agents[0].vel.y - vy;
    sum += ex * ex + ey * ey;
    ++n;
  }
  EXPECT_TRUE(n > 0);
  const double rms = std::sqrt(sum / std::max(1, n));
  std::printf("  velocity RMS = %.4f\n", rms);
  EXPECT_TRUE(rms < 0.15);
}

TEST(velocity_error_rms_below_015_for_sim_pedestrian_speed_and_detection_rate) {
  // sim の歩行者と同じ条件: 1.3 m/s の等速、検出率 0.9(σ=0.05)。最初の 3 s は除外
  std::mt19937 rng(2468);
  std::normal_distribution<double> noise(0.0, 0.05);
  std::uniform_real_distribution<double> uni(0.0, 1.0);
  PeopleTracker tr(TrackerParams{});
  const double speed = 1.3, ang = 0.3, dt = 0.1;
  const double vx = speed * std::cos(ang), vy = speed * std::sin(ang);
  double sum = 0;
  int n = 0;
  for (int k = 0; k < 600; ++k) {
    const double t = k * dt;
    std::vector<Detection> dets;
    if (uni(rng) < 0.9) dets.push_back(makeDet(vx * t + noise(rng), vy * t + noise(rng)));
    tr.update(dets, t);
    if (k < 30) continue;   // 収束待ち
    const auto p = tr.confirmed("odom");
    if (p.agents.size() != 1) continue;   // 欠測で一時的に消えた分は評価しない
    const double ex = p.agents[0].vel.x - vx, ey = p.agents[0].vel.y - vy;
    sum += ex * ex + ey * ey;
    ++n;
  }
  EXPECT_TRUE(n > 300);
  const double rms = std::sqrt(sum / std::max(1, n));
  std::printf("  velocity RMS (1.3 m/s, detection 0.9) = %.4f over %d samples\n", rms, n);
  EXPECT_TRUE(rms < 0.15);
}

TEST(ids_are_kept_when_two_people_cross) {
  std::mt19937 rng(777);
  std::normal_distribution<double> noise(0.0, 0.05);
  PeopleTracker tr(TrackerParams{});
  const double dt = 0.1;
  uint32_t id_a = 0, id_b = 0;   // 十分離れている k=5 で真値に最近傍の track を割り当てて固定する
  for (int k = 0; k <= 40; ++k) {
    const double t = k * dt;
    const Vec2 a{-2.0 + 1.0 * t, -1.0 + 0.5 * t};   // 経路は t=2.4 付近で (0.4,0.2) に交差する
    const Vec2 b{-2.0 + 1.0 * t, 1.4 - 0.5 * t};
    tr.update({makeDet(a.x + noise(rng), a.y + noise(rng)), makeDet(b.x + noise(rng), b.y + noise(rng))}, t);
    if (k < 5) continue;
    const auto p = tr.confirmed("odom");
    EXPECT_TRUE(p.agents.size() == 2);
    if (p.agents.size() != 2) continue;
    auto nearest = [&](Vec2 q) {
      return std::hypot(p.agents[0].pos.x - q.x, p.agents[0].pos.y - q.y) <
                     std::hypot(p.agents[1].pos.x - q.x, p.agents[1].pos.y - q.y)
                 ? p.agents[0].id : p.agents[1].id;
    };
    if (k == 5) {
      id_a = nearest(a);
      id_b = nearest(b);
      EXPECT_TRUE(id_a != id_b);
      continue;
    }
    // 真値同士が重なる区間(間隔 < 0.3 m)は真値側の対応が定まらないので判定しない
    if (std::hypot(a.x - b.x, a.y - b.y) < 0.3) continue;
    EXPECT_TRUE(nearest(a) == id_a);
    EXPECT_TRUE(nearest(b) == id_b);
  }
}

TEST(stationary_person_keeps_single_confirmed_track_with_stable_id) {
  std::mt19937 rng(2024);
  std::normal_distribution<double> noise(0.0, 0.05);
  PeopleTracker tr(TrackerParams{});
  const double dt = 0.1;
  uint32_t id = 0;
  int dup = 0, id_changes = 0;
  for (int k = 0; k < 2000; ++k) {
    tr.update({makeDet(1.0 + noise(rng), 2.0 + noise(rng))}, k * dt);
    const auto p = tr.confirmed("odom");
    if (p.agents.size() > 1) ++dup;
    if (p.agents.size() == 1) {
      if (id != 0 && p.agents[0].id != id) ++id_changes;
      id = p.agents[0].id;
    }
  }
  std::printf("  duplicate cycles = %d, id changes = %d\n", dup, id_changes);
  EXPECT_TRUE(dup == 0);
  EXPECT_TRUE(id_changes == 0);
}

TEST(second_person_appearing_one_meter_away_gets_own_track_without_changing_first_id) {
  std::mt19937 rng(7);
  std::normal_distribution<double> noise(0.0, 0.05);
  PeopleTracker tr(TrackerParams{});
  const double dt = 0.1;
  uint32_t id1 = 0;
  int k = 0;
  for (; k < 50; ++k) tr.update({makeDet(1.0 + noise(rng), 2.0 + noise(rng))}, k * dt);
  {
    const auto p = tr.confirmed("odom");
    EXPECT_TRUE(p.agents.size() == 1);
    if (!p.agents.empty()) id1 = p.agents[0].id;
  }
  bool two = false;
  for (int j = 0; j < 5; ++j, ++k) {
    tr.update({makeDet(1.0 + noise(rng), 2.0 + noise(rng)), makeDet(2.0 + noise(rng), 2.0 + noise(rng))}, k * dt);
    if (tr.confirmed("odom").agents.size() == 2) two = true;
  }
  EXPECT_TRUE(two);
  bool has1 = false;
  for (const auto& a : tr.confirmed("odom").agents) has1 = has1 || a.id == id1;
  EXPECT_TRUE(has1);
}

TEST(ids_are_kept_for_two_people_walking_side_by_side) {
  std::mt19937 rng(31415);
  std::normal_distribution<double> noise(0.0, 0.05);
  PeopleTracker tr(TrackerParams{});
  const double dt = 0.1;
  uint32_t id_a = 0, id_b = 0;
  for (int k = 0; k <= 150; ++k) {
    const double t = k * dt;
    const Vec2 a{1.2 * t, 0.0}, b{1.2 * t, 0.8};   // 0.8 m 間隔で並走
    tr.update({makeDet(a.x + noise(rng), a.y + noise(rng)), makeDet(b.x + noise(rng), b.y + noise(rng))}, t);
    if (k < 5) continue;
    const auto p = tr.confirmed("odom");
    EXPECT_TRUE(p.agents.size() == 2);
    if (p.agents.size() != 2) continue;
    auto nearest = [&](Vec2 q) {
      return std::hypot(p.agents[0].pos.x - q.x, p.agents[0].pos.y - q.y) <
                     std::hypot(p.agents[1].pos.x - q.x, p.agents[1].pos.y - q.y)
                 ? p.agents[0].id : p.agents[1].id;
    };
    if (k == 5) { id_a = nearest(a); id_b = nearest(b); EXPECT_TRUE(id_a != id_b); continue; }
    EXPECT_TRUE(nearest(a) == id_a);
    EXPECT_TRUE(nearest(b) == id_b);
  }
}

TEST(track_is_deleted_after_one_second_without_detections) {
  PeopleTracker tr(TrackerParams{});
  double t = 0;
  for (int k = 0; k < 20; ++k, t += 0.1) tr.update({makeDet(1.0 + 0.5 * t, 0.0)}, t);
  EXPECT_TRUE(tr.confirmed("odom").agents.size() == 1);
  bool alive_at_half = false, gone_after_one = false;
  for (int k = 1; k <= 12; ++k) {
    tr.update({}, t - 0.1 + 0.1 * k);   // 最後の観測は t-0.1
    const double missed = 0.1 * k;
    const auto n = tr.confirmed("odom").agents.size();
    if (std::fabs(missed - 0.5) < 1e-9) alive_at_half = (n == 1);
    if (std::fabs(missed - 1.1) < 1e-9) gone_after_one = (n == 0);
  }
  EXPECT_TRUE(alive_at_half);
  EXPECT_TRUE(gone_after_one);
}

TEST(tracks_below_confirm_hits_are_not_output) {
  TrackerParams prm;
  prm.confirm_hits = 3;
  PeopleTracker tr(prm);
  tr.update({makeDet(1.0, 1.0)}, 0.0);
  EXPECT_TRUE(tr.confirmed("odom").agents.empty());
  tr.update({makeDet(1.0, 1.0)}, 0.1);
  EXPECT_TRUE(tr.confirmed("odom").agents.empty());
  tr.update({makeDet(1.0, 1.0)}, 0.2);
  const auto p = tr.confirmed("odom");
  EXPECT_TRUE(p.agents.size() == 1);
  EXPECT_TRUE(p.frame_id == "odom");
}

TEST(rgbd_detections_are_accepted) {
  PeopleTracker tr(TrackerParams{});
  for (int k = 0; k < 6; ++k) tr.update({makeDet(2.0, -1.0, DetectionSource::RGBD)}, 0.1 * k);  // 既定の confirm_hits(5)を超える回数
  const auto p = tr.confirmed("odom");
  EXPECT_TRUE(p.agents.size() == 1);
  if (p.agents.size() == 1) EXPECT_NEAR(p.agents[0].pos.x, 2.0, 0.05);
}

namespace {
double stationaryVelRms(const TrackerParams& prm) {
  std::mt19937 rng(99);
  std::normal_distribution<double> noise(0.0, 0.05);
  PeopleTracker tr(prm);
  double sum = 0;
  int n = 0;
  for (int k = 0; k < 2000; ++k) {
    tr.update({makeDet(1.0 + noise(rng), 2.0 + noise(rng))}, k * 0.1);
    if (k < 30) continue;
    const auto p = tr.confirmed("odom");
    if (p.agents.size() != 1) continue;
    sum += p.agents[0].vel.x * p.agents[0].vel.x + p.agents[0].vel.y * p.agents[0].vel.y;
    ++n;
  }
  return std::sqrt(sum / std::max(1, n));
}

}  // namespace

TEST(stationary_velocity_rms_below_015_with_default_params) {
  const double r = stationaryVelRms(TrackerParams{});
  std::printf("  stationary vel RMS default = %.4f\n", r);
  EXPECT_TRUE(r < 0.15);
}

MINI_TEST_MAIN()
