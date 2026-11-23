// PedModel(sfm2)の単体テスト。数値は帯で持つ。乱数は専用 seed で決定的。
#include <algorithm>
#include <cmath>
#include <vector>

#include "mini_test.hpp"
#include "sim/ped_model.hpp"
#include "sim/ttc_ped_model.hpp"
#include "sim/orca_ped_model.hpp"

using namespace social_nav;
using namespace social_nav::sim;

namespace {
Pedestrian mk(uint32_t id, Vec2 pos, Vec2 goal, double v0 = 1.3) {
  Pedestrian p;
  p.id = id;
  p.pos = pos;
  p.home = pos;
  p.goal = goal;
  p.v0 = v0;
  const double d = std::hypot(goal.x - pos.x, goal.y - pos.y);
  p.vel = {v0 * (goal.x - pos.x) / d, v0 * (goal.y - pos.y) / d};
  return p;
}
double dist(const Pedestrian& a, const Pedestrian& b) { return std::hypot(a.pos.x - b.pos.x, a.pos.y - b.pos.y); }
}  // namespace

TEST(ped_free_flow_speed_in_band) {
  PedModel m(PedParams{}, {}, 4243);
  std::vector<Pedestrian> peds{mk(1, {0, 0}, {100, 0}, 1.3)};
  double sum = 0;
  int cnt = 0;
  for (int k = 0; k < 1000; ++k) {   // 20 s
    m.step(peds, nullptr, 0.02);
    if (k >= 250) { sum += std::hypot(peds[0].vel.x, peds[0].vel.y); ++cnt; }
  }
  EXPECT_NEAR(sum / cnt, 1.3, 1.3 * 0.05);
}

TEST(ped_head_on_no_contact_and_early_avoid) {
  PedModel m(PedParams{}, {}, 4243);
  std::vector<Pedestrian> peds{mk(1, {-6, 0}, {6, 0}), mk(2, {6, 0}, {-6, 0})};
  double min_d = 1e9, avoid_start = -1;
  for (int k = 0; k < 1000; ++k) {
    m.step(peds, nullptr, 0.02);
    const double d = dist(peds[0], peds[1]);
    min_d = std::min(min_d, d);
    if (avoid_start < 0 && std::fabs(peds[0].pos.y) > 0.05) avoid_start = d;
  }
  EXPECT_TRUE(min_d > 0.6);          // 外形(半径 0.3 x2)が接しない
  EXPECT_TRUE(avoid_start >= 2.0);   // 2 m 以上手前で横にずれ始める
}

TEST(ped_force_depends_on_relative_velocity) {
  PedModel m(PedParams{}, {}, 1);
  // 同じ位置関係(b は a の 1.5 m 前方)。接近中 vs 離反中
  const Vec2 pa{0, 0}, pb{1.5, 0.2};
  const Vec2 f_app = m.interactionForce(pa, {1.3, 0}, pb, {-1.3, 0});
  const Vec2 f_sep = m.interactionForce(pa, {1.3, 0}, pb, {1.3, 0});
  EXPECT_TRUE(std::hypot(f_app.x, f_app.y) > 1.5 * std::hypot(f_sep.x, f_sep.y));
  EXPECT_TRUE(f_app.x < 0.0);   // 接近中は減速方向に押す
}

TEST(ped_same_seed_same_result_other_seed_differs) {
  auto run = [](uint32_t seed) {
    PedModel m(PedParams{}, {}, seed);
    std::vector<Pedestrian> peds{mk(1, {0, 0}, {20, 0}), mk(2, {20, 0.3}, {0, 0})};
    for (int k = 0; k < 300; ++k) m.step(peds, nullptr, 0.02);
    return peds;
  };
  const auto a = run(5), b = run(5), c = run(6);
  EXPECT_TRUE(a[0].pos.x == b[0].pos.x && a[0].pos.y == b[0].pos.y && a[1].pos.y == b[1].pos.y);
  EXPECT_TRUE(a[0].pos.y != c[0].pos.y);
}

TEST(ped_works_with_robot_and_without) {
  PedModel m(PedParams{}, {}, 7);
  std::vector<Pedestrian> peds{mk(1, {-3, 0}, {3, 0})};
  const RobotBody robot{{0.0, 0.0}, {-0.5, 0.0}, 0.3};
  for (int k = 0; k < 300; ++k) m.step(peds, &robot, 0.02);
  EXPECT_TRUE(std::isfinite(peds[0].pos.x) && std::isfinite(peds[0].pos.y));
  EXPECT_TRUE(std::hypot(peds[0].pos.x - robot.pos.x, peds[0].pos.y - robot.pos.y) > 0.6);   // ロボットと重ならない
  PedModel m2(PedParams{}, {}, 7);
  std::vector<Pedestrian> q{mk(1, {-3, 0}, {3, 0})};
  for (int k = 0; k < 300; ++k) m2.step(q, nullptr, 0.02);
  EXPECT_TRUE(q[0].pos.x > 0.0);   // ロボットを無視して通過
}

TEST(ped_stops_at_goal_without_drift) {
  PedModel m(PedParams{}, {}, 3);
  std::vector<Pedestrian> peds{mk(1, {0, 0}, {2, 0}), mk(2, {3.0, 0.0}, {3.0, 0.0}, 0.0)};   // 凍結者は目標の 1 m 先(調整 2 で斥力が強まり、0.4 m 先だと目標半径に入れない)
  peds[1].frozen = true;
  peds[1].vel = {0, 0};
  for (int k = 0; k < 600; ++k) m.step(peds, nullptr, 0.02);
  const Vec2 p0 = peds[0].pos;
  for (int k = 0; k < 300; ++k) m.step(peds, nullptr, 0.02);
  EXPECT_TRUE(std::hypot(peds[0].pos.x - p0.x, peds[0].pos.y - p0.y) < 1e-9);
  EXPECT_TRUE(std::hypot(peds[0].vel.x, peds[0].vel.y) < 1e-9);
}

TEST(ped_wall_keeps_inside) {
  const std::vector<Segment> walls{{{-5, 1.0}, {5, 1.0}}};
  PedModel m(PedParams{}, walls, 3);
  std::vector<Pedestrian> peds{mk(1, {0, 0}, {0, 3}, 1.3)};   // 壁へ向かう
  for (int k = 0; k < 500; ++k) m.step(peds, nullptr, 0.02);
  EXPECT_TRUE(peds[0].pos.y < 1.0);
}

TEST(ped_family_stays_together) {
  PedModel m(PedParams{}, {}, 11);
  const double v0s[3] = {1.0, 1.3, 1.2};   // 速度の違う 3 人でもばらけない
  std::vector<Pedestrian> peds;
  for (int i = 0; i < 3; ++i) {
    Pedestrian p = mk(i + 1, {0.0, 0.8 * i}, {100, 0.8 * i}, v0s[i]);
    p.kind = 1;
    p.group_id = 7;
    peds.push_back(p);
  }
  double sum = 0;
  int cnt = 0;
  for (int k = 0; k < 1500; ++k) {   // 30 s
    m.step(peds, nullptr, 0.02);
    sum += (dist(peds[0], peds[1]) + dist(peds[1], peds[2]) + dist(peds[0], peds[2])) / 3.0;
    ++cnt;
  }
  EXPECT_TRUE(sum / cnt <= 1.5);
  EXPECT_TRUE(dist(peds[0], peds[1]) > 0.4);   // 重ならない
}

// 屋内の群れ: group_id >= kPaperGazeGroupIdMin の群れは、横に並んで 6 s 歩いても横の間隔が 0.4 m 以上に保たれる(視線が減速の形のため)
TEST(ped_group_paper_gaze_keeps_side_by_side) {
  for (uint32_t seed : {11u, 12u, 13u}) {
    PedModel m(PedParams{}, {}, seed);
    std::vector<Pedestrian> peds;
    for (int i = 0; i < 2; ++i) {
      Pedestrian p = mk(i + 1, {10.0, 0.75 * i}, {-4.0, 0.75 * i}, 1.3);
      p.group_id = kPaperGazeGroupIdMin;
      peds.push_back(p);
    }
    for (int k = 0; k < 300; ++k) m.step(peds, nullptr, 0.02);   // 6 s
    EXPECT_TRUE(std::fabs(peds[1].pos.y - peds[0].pos.y) >= 0.4);
  }
}

// 屋内の群れ: group_id < kPaperGazeGroupIdMin(crowd の 0..2)の視線は従来の式のまま(重心側へ向ける横加速度)。100 以上は速度に沿う減速
TEST(ped_group_gaze_form_by_group_id) {
  auto run = [](int gid) {
    PedParams prm;
    prm.noise_sigma = 0.0;   // 雑音を止め、人同士の相互作用も 0 にして、視線項だけの加速度を見る
    prm.A = 0.0;
    PedModel m(prm, {}, 5);
    std::vector<Pedestrian> peds{mk(1, {0.0, 0.0}, {100.0, 0.0}, 1.3), mk(2, {0.0, 1.0}, {100.0, 1.0}, 1.3)};   // 相手は真横(重心は進行方向から 90°。視野 +-50° の外)
    peds[0].group_id = gid;
    peds[1].group_id = gid;
    m.step(peds, nullptr, 0.02);
    return peds[0].vel;
  };
  const Vec2 old_form = run(0), paper = run(kPaperGazeGroupIdMin);
  EXPECT_TRUE(old_form.y > 1e-3 && std::fabs(old_form.x - 1.3) < 1e-6);   // 横(重心側)へ曲げる。速さは保つ
  EXPECT_TRUE(paper.x < 1.3 - 1e-3 && std::fabs(paper.y) < 1e-9);        // 進行方向に減速。横には曲げない
}

TEST(ped_group_force_only_within_group) {
  auto run = [](int gid_b) {
    PedModel m(PedParams{}, {}, 11);
    std::vector<Pedestrian> peds{mk(1, {0, 0}, {100, 0}), mk(2, {0, 3.0}, {100, 3.0})};
    peds[0].group_id = 5;
    peds[1].group_id = gid_b;
    for (int k = 0; k < 500; ++k) m.step(peds, nullptr, 0.02);
    return dist(peds[0], peds[1]);
  };
  EXPECT_TRUE(run(5) < run(-1) - 0.5);   // 同群だけ引き合う(3 m 離れた同速度の 2 人)
}

TEST(ped_phone_is_slower_and_avoids_later) {
  auto freeSpeed = [](int kind) {
    PedModel m(PedParams{}, {}, 4243);
    std::vector<Pedestrian> peds{mk(1, {0, 0}, {100, 0}, 1.3)};
    peds[0].kind = kind;
    double sum = 0;
    int cnt = 0;
    for (int k = 0; k < 1000; ++k) {
      m.step(peds, nullptr, 0.02);
      if (k >= 250) { sum += std::hypot(peds[0].vel.x, peds[0].vel.y); ++cnt; }
    }
    return sum / cnt;
  };
  const double ratio = freeSpeed(2) / freeSpeed(0);
  EXPECT_TRUE(ratio > 0.72 && ratio < 0.88);   // v0 x0.8 を中心に帯
  auto avoidStart = [](int kind) {
    PedParams prm;
    prm.noise_sigma = 0.0;   // 雑音の揺れを回避開始と取り違えない
    PedModel m(prm, {}, 4243);
    std::vector<Pedestrian> peds{mk(1, {-6, 0}, {6, 0}), mk(2, {6, 0}, {-6, 0})};
    for (auto& p : peds) { p.kind = kind; if (kind == 2) p.attention = 1.0; }
    for (int k = 0; k < 1000; ++k) {
      m.step(peds, nullptr, 0.02);
      if (std::fabs(peds[0].pos.y) > 0.05) return dist(peds[0], peds[1]);
    }
    return 0.0;
  };
  const double normal = avoidStart(0), phone = avoidStart(2);
  EXPECT_TRUE(normal >= 2.0);
  EXPECT_TRUE(phone < normal - 0.5);   // 回避の開始が遅い(近づいてから避ける)
}

TEST(ped_rear_weight_applies_outside_fov) {
  PedParams prm;
  prm.noise_sigma = 0.0;
  prm.rear_weight = 0.65;
  PedModel m(prm, {}, 1);
  // 同速で前後 1 m(同じ向き)。後続の相手は前方(視野内)、先行の相手は後方(視野外)
  std::vector<Pedestrian> peds{mk(1, {0, 0}, {100, 0}), mk(2, {1, 0}, {101, 0})};
  const Vec2 v0 = peds[0].vel;
  m.step(peds, nullptr, 0.02);
  const double slow = v0.x - peds[0].vel.x;    // 後続の減速
  const double fast = peds[1].vel.x - v0.x;    // 先行の加速
  EXPECT_TRUE(slow > 0.0);
  EXPECT_NEAR(fast / slow, prm.rear_weight, 1e-6);
}

// ---- TtcPedModel ----
TEST(ttc_head_on_force_points_away_laterally) {
  TtcPedModel m(TtcParams{}, {}, 1);
  // a は +x へ、b は -x へ。b は a の進路から +y に 0.1 m ずれている -> a は -y(b から離れる向き)へ押され、減速もする
  const Vec2 f = m.interactionForce({0, 0}, {1.3, 0}, 0.3, {3, 0.1}, {-1.3, 0}, 0.3);
  EXPECT_TRUE(f.y < 0.0);
  EXPECT_TRUE(f.x < 0.0);
  // b から見ると対称に +y へ押される
  const Vec2 g = m.interactionForce({3, 0.1}, {-1.3, 0}, 0.3, {0, 0}, {1.3, 0}, 0.3);
  EXPECT_TRUE(g.y > 0.0);
}

TEST(ttc_infinite_tau_gives_zero_force) {
  TtcPedModel m(TtcParams{}, {}, 1);
  // 離れていく(相対速度が遠ざかる向き)
  Vec2 f = m.interactionForce({0, 0}, {-1.0, 0}, 0.3, {2, 0}, {1.0, 0}, 0.3);
  EXPECT_NEAR(f.x, 0.0, 1e-12);
  EXPECT_NEAR(f.y, 0.0, 1e-12);
  // かすらない(横に 2 m ずれて平行)
  f = m.interactionForce({0, 0}, {1.0, 0}, 0.3, {5, 2}, {-1.0, 0}, 0.3);
  EXPECT_NEAR(f.x, 0.0, 1e-12);
  EXPECT_NEAR(f.y, 0.0, 1e-12);
  // 相対速度 0
  f = m.interactionForce({0, 0}, {1.0, 0}, 0.3, {2, 0}, {1.0, 0}, 0.3);
  EXPECT_NEAR(f.x, 0.0, 1e-12);
  // 重なり: 相互作用項は 0
  f = m.interactionForce({0, 0}, {1.0, 0}, 0.3, {0.4, 0}, {-1.0, 0}, 0.3);
  EXPECT_NEAR(f.x, 0.0, 1e-12);
}

TEST(ttc_force_capped) {
  TtcParams prm;
  TtcPedModel m(prm, {}, 1);
  const Vec2 f = m.interactionForce({0, 0}, {1.3, 0}, 0.3, {0.7, 0.05}, {-1.3, 0}, 0.3);
  EXPECT_TRUE(std::hypot(f.x, f.y) <= prm.f_max + 1e-9);
}

TEST(ttc_same_seed_reproduces_and_head_on_passes) {
  auto run = [](uint32_t seed, double& min_d) {
    TtcPedModel m(TtcParams{}, {}, seed);
    std::vector<Pedestrian> peds{mk(1, {-6, 0}, {6, 0}), mk(2, {6, 0}, {-6, 0})};
    min_d = 1e9;
    for (int k = 0; k < 1000; ++k) {
      m.step(peds, nullptr, 0.02);
      min_d = std::min(min_d, dist(peds[0], peds[1]));
    }
    return peds;
  };
  double d1, d2, d3;
  const auto a = run(77, d1), b = run(77, d2), c = run(78, d3);
  EXPECT_TRUE(a[0].pos.x == b[0].pos.x && a[0].pos.y == b[0].pos.y && d1 == d2);
  EXPECT_TRUE(a[0].pos.y != c[0].pos.y);
  EXPECT_TRUE(d1 > 0.5);
}

TEST(orca_head_on_avoids_without_contact) {
  OrcaPedModel m(OrcaParams{}, {});
  // 完全な正面(横ずれ 0)でも、ORCA は一方の側へ避ける(解は対称でなく LP が選ぶ)
  std::vector<Pedestrian> peds{mk(1, {-6, 0}, {6, 0}), mk(2, {6, 0.05}, {-6, 0})};
  double min_d = 1e9, max_y = 0;
  for (int k = 0; k < 1000; ++k) {
    m.step(peds, nullptr, 0.02);
    min_d = std::min(min_d, dist(peds[0], peds[1]));
    max_y = std::max(max_y, std::fabs(peds[0].pos.y));
  }
  EXPECT_TRUE(min_d >= 0.6 - 1e-6);   // 外形が接しない
  EXPECT_TRUE(max_y > 0.05);          // 横に避けた
}

TEST(orca_receding_pair_keeps_velocity) {
  OrcaPedModel m(OrcaParams{}, {});
  // 同じ向きに同じ速さで離れていく。1 人目の前に 2 人目がいるが追いつかない
  std::vector<Pedestrian> peds{mk(1, {0, 0}, {50, 0}, 1.3), mk(2, {3, 0}, {50, 0}, 1.3)};
  m.step(peds, nullptr, 0.02);
  EXPECT_NEAR(peds[0].vel.x, 1.3, 1e-9);
  EXPECT_NEAR(peds[0].vel.y, 0.0, 1e-9);
  // 後ろ向きに離れていく相手(2 人目の目標は反対側)
  std::vector<Pedestrian> q{mk(1, {0, 0}, {50, 0}, 1.3), mk(2, {-3, 0}, {-50, 0}, 1.3)};
  m.step(q, nullptr, 0.02);
  EXPECT_NEAR(q[0].vel.x, 1.3, 1e-9);
  EXPECT_NEAR(q[0].vel.y, 0.0, 1e-9);
}

TEST(orca_does_not_cross_wall) {
  // 壁(y = 0.9)に向かう目標へ、壁際を歩く。壁を越えない
  const std::vector<Segment> walls{{{-2, 0.9}, {30, 0.9}}, {{-2, -0.9}, {30, -0.9}}};
  OrcaPedModel m(OrcaParams{}, walls);
  std::vector<Pedestrian> peds{mk(1, {0, 0.5}, {20, 3.0}), mk(2, {20, -0.5}, {0, -4.0}), mk(3, {10, 0.0}, {10, 0.85}, 1.0)};
  double max_y = 0, min_y = 0;
  for (int k = 0; k < 1500; ++k) {
    m.step(peds, nullptr, 0.02);
    for (const Pedestrian& p : peds) { max_y = std::max(max_y, p.pos.y); min_y = std::min(min_y, p.pos.y); }
  }
  EXPECT_TRUE(max_y <= 0.6 + 1e-3 && min_y >= -0.6 - 1e-3);   // 外形(半径 0.3)が壁(±0.9)に入らない
}

TEST(orca_oblique_approach_to_wall_stays_outside) {
  // 壁 y = 0 へ斜めに向かう速度(1.163, 0.581)。右脚の向きの誤りがあると線ができず壁へ入る
  const std::vector<Segment> walls{{{-10, 0}, {10, 0}}};
  OrcaPedModel m(OrcaParams{}, walls);
  std::vector<Pedestrian> peds{mk(1, {0, -1}, {10, 5})};
  peds[0].vel = {1.163, 0.581};
  double max_y = -1e9;
  for (int k = 0; k < 300; ++k) {
    m.step(peds, nullptr, 0.02);
    max_y = std::max(max_y, peds[0].pos.y);
  }
  EXPECT_TRUE(max_y <= -0.3 + 1e-3);
}

TEST(orca_wall_end_projection_exactly_one_inside_radius_is_finite) {
  // 壁 (0,0)-(1,0) の端(0,0)の真上 0.1 m に立つ。辺への射影 s がちょうど 1 で半径(0.3)内。
  // s < 1.0 だと重なり判定を外れ、sqrt(d - r^2) が負の数になって NaN の線ができ(LP が無視する)、壁へ入る(RVO2 は s <= 1.0)
  const std::vector<Segment> walls{{{0, 0}, {1, 0}}};
  OrcaPedModel m(OrcaParams{}, walls);
  std::vector<Pedestrian> peds{mk(1, {0, 0.1}, {2, -3})};   // 目標は壁の向こう側
  m.step(peds, nullptr, 0.02);
  EXPECT_TRUE(std::isfinite(peds[0].vel.x) && std::isfinite(peds[0].vel.y));
  EXPECT_TRUE(peds[0].vel.y >= -1e-9);   // 壁(y = 0)の向こう側へ入らない
}

TEST(orca_robot_takes_no_share_and_ignore_works) {
  OrcaPedModel m(OrcaParams{}, {});
  RobotBody robot{{3, 0.0}, {-1.0, 0}, 0.3};
  std::vector<Pedestrian> a{mk(1, {0, 0}, {20, 0})};
  m.step(a, &robot, 0.02);
  EXPECT_TRUE(std::fabs(a[0].vel.y) > 1e-3 || a[0].vel.x < 1.3 - 1e-3);   // ロボットを避ける
  std::vector<Pedestrian> b{mk(1, {0, 0}, {20, 0})};
  m.step(b, nullptr, 0.02);
  EXPECT_NEAR(b[0].vel.x, 1.3, 1e-9);
  EXPECT_NEAR(b[0].vel.y, 0.0, 1e-9);
}

TEST(orca_same_input_reproduces) {
  auto run = []() {
    OrcaPedModel m(OrcaParams{}, {});
    std::vector<Pedestrian> peds{mk(1, {-6, 0}, {6, 0}), mk(2, {6, 0}, {-6, 0}), mk(3, {0, -6}, {0, 6})};
    for (int k = 0; k < 800; ++k) m.step(peds, nullptr, 0.02);
    return peds;
  };
  const auto a = run(), b = run();
  bool same = true;
  for (std::size_t i = 0; i < a.size(); ++i) same = same && a[i].pos.x == b[i].pos.x && a[i].pos.y == b[i].pos.y;
  EXPECT_TRUE(same);
}

// ---- 壁の角を 2 回数えない ----
namespace {
// 1 歩だけ進めたあとの速度。3 モデル(sfm2 / ttc / legacy)で同じ入力を与える
std::vector<Vec2> oneStepVel(const std::vector<Segment>& walls, Vec2 pos, Vec2 goal) {
  std::vector<Vec2> out;
  {
    PedModel m(PedParams{}, walls, 7);
    std::vector<Pedestrian> peds{mk(1, pos, goal, 1.3)};
    m.step(peds, nullptr, 0.02);
    out.push_back(peds[0].vel);
  }
  {
    TtcPedModel m(TtcParams{}, walls, 7);
    std::vector<Pedestrian> peds{mk(1, pos, goal, 1.3)};
    m.step(peds, nullptr, 0.02);
    out.push_back(peds[0].vel);
  }
  {
    SocialForceWorld m(SfmParams{}, walls);
    std::vector<Pedestrian> peds{mk(1, pos, goal, 1.3)};
    m.step(peds, nullptr, 0.02);
    out.push_back(peds[0].vel);
  }
  return out;
}
void addSideOpeningLocal(std::vector<Segment>& walls, double x0, double x1, double hw, double xc, double w, double ye) {
  for (const double sg : {1.0, -1.0}) {
    walls.push_back({{x0, sg * hw}, {xc - 0.5 * w, sg * hw}});
    walls.push_back({{xc + 0.5 * w, sg * hw}, {x1, sg * hw}});
    walls.push_back({{xc - 0.5 * w, sg * hw}, {xc - 0.5 * w, sg * ye}});
    walls.push_back({{xc + 0.5 * w, sg * hw}, {xc + 0.5 * w, sg * ye}});
    walls.push_back({{xc - 0.5 * w, sg * ye}, {xc + 0.5 * w, sg * ye}});
  }
}
}  // namespace

TEST(wall_shared_corner_counted_once) {
  // 横の壁 w1 の右端 (0,1) と縦の壁 w2 の下端 (0,1) が同じ角。(0.5,0.5) の最近点はどちらの線分でもその角
  const Segment w1{{-5, 1.0}, {0, 1.0}}, w2{{0, 1.0}, {0, 5.0}};
  const std::vector<Segment> both{w1, w2};
  EXPECT_TRUE(!isRepeatedWallCorner({0.5, 0.5}, both, 0, {0, 1.0}));
  EXPECT_TRUE(isRepeatedWallCorner({0.5, 0.5}, both, 1, {0, 1.0}));
  // 力は w1 だけのときと同じ(角が 1 回)。3 モデルとも
  const std::vector<Vec2> v_both = oneStepVel(both, {0.5, 0.5}, {3.0, 3.0});
  const std::vector<Vec2> v_one = oneStepVel({w1}, {0.5, 0.5}, {3.0, 3.0});
  for (std::size_t k = 0; k < 3; ++k) {
    EXPECT_TRUE(v_both[k].x == v_one[k].x && v_both[k].y == v_one[k].y);
  }
}

TEST(wall_unshared_endpoint_counts_as_before) {
  // w2 の下端が (0.3,1) で w1 の端 (0,1) と別。(0.5,0.5) の最近点は w1 が (0,1)、w2 が (0.3,1)。2 つとも数える
  const Segment w1{{-5, 1.0}, {0, 1.0}}, w2{{0.3, 1.0}, {0.3, 5.0}};
  const std::vector<Segment> both{w1, w2};
  EXPECT_TRUE(!isRepeatedWallCorner({0.5, 0.5}, both, 0, {0, 1.0}));
  EXPECT_TRUE(!isRepeatedWallCorner({0.5, 0.5}, both, 1, {0.3, 1.0}));
  const std::vector<Vec2> v_both = oneStepVel(both, {0.5, 0.5}, {3.0, 3.0});
  const std::vector<Vec2> v_one = oneStepVel({w1}, {0.5, 0.5}, {3.0, 3.0});
  for (std::size_t k = 0; k < 3; ++k) EXPECT_TRUE(v_both[k].y != v_one[k].y);
  // 線分の内側に最近点がある(端点でない)ときは、同じ点が重なっても飛ばさない
  const std::vector<Segment> same{{{-5, 1.0}, {5, 1.0}}, {{-5, 1.0}, {5, 1.0}}};
  EXPECT_TRUE(!isRepeatedWallCorner({0.5, 0.5}, same, 1, {0.5, 1.0}));
}

TEST(wall_opening_corners_each_counted_once) {
  // 実際の開口部(addSideOpening 相当)の角 4 つ(左右 × 上下)が、人の最近点になるとき 1 回ずつ数えられる。
  // 廊下の壁の端 a + 1.0 * (b - a) は丸めで b と一致しないことがあり、== の比較では左の角が 2 回数えのままだった。
  // 旧幅の 2 件(0.917、1.024)に加えて幅を振って調べる
  int checked_left = 0;
  for (double w : {0.917, 1.024, 1.1, 1.2, 1.3, 1.4, 1.5, 1.6})
    for (double hw : {1.75, 1.0}) {
      std::vector<Segment> walls;
      addSideOpeningLocal(walls, -6.0, 24.0, hw, 3.0, w, 9.0);
      for (const double sg : {1.0, -1.0}) {
        const Vec2 p{3.0, sg * (hw - 0.5)};   // 廊下側で開口部の中心の前。左右の角が最近点
        std::vector<Vec2> qs;
        for (const Segment& s : walls) qs.push_back(nearestPointOnSegment(p, s));
        for (const double cx : {3.0 - 0.5 * w, 3.0 + 0.5 * w}) {
          const Vec2 corner{cx, sg * hw};
          int counted = 0, touching = 0;
          for (std::size_t i = 0; i < walls.size(); ++i) {
            if (std::abs(qs[i].x - corner.x) > 1e-9 || std::abs(qs[i].y - corner.y) > 1e-9) continue;
            ++touching;
            if (!isRepeatedWallCorner(p, walls, i, qs[i])) ++counted;
          }
          EXPECT_TRUE(touching == 2);   // 廊下の壁と枝の壁
          EXPECT_TRUE(counted == 1);
          if (cx < 3.0) ++checked_left;
        }
      }
    }
  EXPECT_TRUE(checked_left == 32);
}

TEST(wall_opening_single_ped_crosses_old_widths) {
  // 壁の角の二重計上: 開口部を通れなかった旧幅の 2 件。side_door のシード 16(w 0.917 / v0 1.141 / y0 5.55)と
  // シード 19(w 1.024 / v0 1.103 / y0 5.27)の配置。横切る経路で、反対側の開口部の手前の角(2 本の線分が共有)で止まっていた。
  // 人が単独で(雑音の種は配置の検査と同じ seed + 4242)、60 s 以内に目標の 0.5 m 以内へ着く
  const struct { double w, v0, y0; uint32_t seed; } cases[] = {{0.917, 1.141, 5.55, 16}, {1.024, 1.103, 5.27, 19}};
  for (const auto& c : cases) {
    std::vector<Segment> walls;
    addSideOpeningLocal(walls, -6.0, 24.0, 1.75, 3.0, c.w, 9.0);
    PedModel m(PedParams{}, walls, c.seed + 4242u);
    const Vec2 goal{3.0, -6.0};
    std::vector<Pedestrian> peds{mk(1, {3.0, c.y0}, goal, c.v0)};
    bool reached = false;
    for (int k = 0; k < 3000 && !reached; ++k) {
      m.step(peds, nullptr, 0.02);
      reached = std::hypot(peds[0].pos.x - goal.x, peds[0].pos.y - goal.y) < 0.5;
    }
    EXPECT_TRUE(reached);
  }
}

int main() { return mini_test::runAll(); }
