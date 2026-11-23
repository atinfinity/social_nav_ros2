// シナリオ一覧の単体テスト。既定の 6 本と順序は変えない。後方からの人流 3 本と屋内 3 本は別の一覧で、記録のみ(屋内は door_oncoming、side_door、group_oncoming の 3 本)。
#include <algorithm>
#include <cmath>
#include <random>

#include "mini_test.hpp"
#include "sim/ped_event.hpp"
#include "sim/ped_model.hpp"
#include "sim/orca_ped_model.hpp"
#include "sim/ttc_ped_model.hpp"
#include "sim/scenarios.hpp"

using namespace social_nav;
using namespace social_nav::sim;

TEST(default_scenarios_unchanged) {
  const std::vector<std::string> expected{"corridor_oncoming", "crossing", "overtake", "follow", "static_group", "crowd"};
  EXPECT_TRUE(scenarioNames() == expected);
}

TEST(rear_scenarios_separate_list) {
  const std::vector<std::string> expected{"rear_follow", "rear_overtake", "rear_overtake_flow"};
  EXPECT_TRUE(rearScenarioNames() == expected);
  for (const std::string& n : rearScenarioNames())
    for (const std::string& m : scenarioNames()) EXPECT_TRUE(n != m);
}

TEST(rear_scenarios_definition) {
  struct Case { const char* name; std::size_t n; double v_lo, v_hi; };
  const Case cases[] = {{"rear_follow", 1, 0.5, 1.3}, {"rear_overtake", 1, 0.5, 2.1}, {"rear_overtake_flow", 3, 0.5, 2.1}};
  for (const Case& c : cases)
    for (unsigned seed = 1; seed <= 20; ++seed) {
      std::mt19937 rng(seed);
      const Scenario s = makeScenario(c.name, rng);
      EXPECT_TRUE(s.name == c.name);
      EXPECT_TRUE(s.peds.size() == c.n);
      for (const PedInit& p : s.peds) {
        EXPECT_TRUE(p.pos.x < s.start.x - 1.5);      // ロボットの後方から出発
        EXPECT_TRUE(p.goal.x > p.pos.x);              // 同じ向き(+x)に進む
        EXPECT_NEAR(p.goal.y, p.pos.y, 1e-12);
        EXPECT_TRUE(p.v0 >= c.v_lo && p.v0 <= c.v_hi);
        EXPECT_TRUE(!p.frozen && !p.loop);
        EXPECT_TRUE(std::fabs(p.pos.y) < 1.75);
        for (const Segment& w : s.walls) EXPECT_TRUE(p.pos.x >= std::min(w.a.x, w.b.x));
      }
    }
}

TEST(rear_scenarios_reproducible) {
  for (const std::string& n : rearScenarioNames()) {
    std::mt19937 a(7), b(7);
    const Scenario x = makeScenario(n, a), y = makeScenario(n, b);
    EXPECT_TRUE(x.peds.size() == y.peds.size());
    for (std::size_t i = 0; i < x.peds.size(); ++i) {
      EXPECT_TRUE(x.peds[i].pos.x == y.peds[i].pos.x && x.peds[i].pos.y == y.peds[i].pos.y);
      EXPECT_TRUE(x.peds[i].v0 == y.peds[i].v0);
    }
  }
}

// ---- 屋内 3 本 ----
namespace {

bool segmentsIntersect(Vec2 a, Vec2 b, Vec2 c, Vec2 d) {
  auto cross = [](Vec2 o, Vec2 p, Vec2 q) { return (p.x - o.x) * (q.y - o.y) - (p.y - o.y) * (q.x - o.x); };
  const double d1 = cross(c, d, a), d2 = cross(c, d, b), d3 = cross(a, b, c), d4 = cross(a, b, d);
  return ((d1 > 0) != (d2 > 0)) && ((d3 > 0) != (d4 > 0)) && d1 != 0 && d2 != 0 && d3 != 0 && d4 != 0;
}

// 開口部の幅(door: x = 5 の仕切りの隙間、side: 開口部の隙間)。壁の端点から求める
double doorWidth(const Scenario& s) {
  double top = 1e9, bot = -1e9;   // x = 5 の壁の y 端(0 に最も近いもの)
  for (const Segment& w : s.walls)
    if (w.a.x == 5.0 && w.b.x == 5.0) {
      const double lo = std::min(w.a.y, w.b.y), hi = std::max(w.a.y, w.b.y);
      if (lo > 0) top = std::min(top, lo);
      if (hi < 0) bot = std::max(bot, hi);
    }
  return top - bot;
}

// 壁の内側にいる点(廊下 |y| < 1.75 か、side_door の枝通路)。clearance は壁からの距離
bool insideSideDoor(Vec2 p, double w) {
  if (std::fabs(p.y) < 1.75 && p.x > -6.0 && p.x < 24.0) return true;
  return std::fabs(p.x - 3.0) < 0.5 * w && std::fabs(p.y) < 9.0;
}

// main.cpp のループと同じ via の切り替え
void switchVia(const PedInit& pi, Pedestrian& p, bool& done) {
  if (!pi.has_via || done) return;
  const double px = p.pos.x - pi.via.x, py = p.pos.y - pi.via.y;
  if (std::hypot(px, py) < 0.5 || px * (pi.via.x - pi.pos.x) + py * (pi.via.y - pi.pos.y) > 0.0) {
    done = true;
    p.goal = pi.goal;
  }
}

std::vector<Pedestrian> makePeds(const Scenario& s) {
  std::vector<Pedestrian> peds;
  for (std::size_t i = 0; i < s.peds.size(); ++i) {
    const PedInit& pi = s.peds[i];
    const Vec2 g = pi.has_via ? pi.via : pi.goal;
    Pedestrian p;
    p.id = static_cast<uint32_t>(i + 1);
    p.pos = pi.pos; p.home = pi.pos; p.goal = g; p.v0 = pi.v0; p.radius = 0.3; p.group_id = pi.group_id; p.frozen = pi.frozen || pi.release_t > 0.0;
    const double d = std::hypot(g.x - pi.pos.x, g.y - pi.pos.y);
    if (!pi.frozen && !(pi.release_t > 0.0) && d > 1e-9) p.vel = {pi.v0 * (g.x - pi.pos.x) / d, pi.v0 * (g.y - pi.pos.y) / d};
    peds.push_back(p);
  }
  return peds;
}

}  // namespace

TEST(indoor_scenarios_separate_list) {
  const std::vector<std::string> expected{"door_oncoming", "side_door", "group_oncoming"};
  EXPECT_TRUE(indoorScenarioNames() == expected);
  for (const std::string& n : indoorScenarioNames()) {
    for (const std::string& m : scenarioNames()) EXPECT_TRUE(n != m);
    for (const std::string& m : rearScenarioNames()) EXPECT_TRUE(n != m);
  }
  EXPECT_TRUE(scenarioNames().size() == 6 && rearScenarioNames().size() == 3);
}

TEST(indoor_door_oncoming_definition) {
  for (unsigned seed = 1; seed <= 20; ++seed) {
    std::mt19937 rng(seed);
    const Scenario s = makeScenario("door_oncoming", rng);
    EXPECT_TRUE(s.peds.size() == 1);
    const PedInit& p = s.peds[0];
    const double w = doorWidth(s);
    EXPECT_TRUE(w >= 0.9 - 1e-9 && w <= 1.2 + 1e-9);
    EXPECT_TRUE(p.v0 >= 0.5 && p.v0 <= 2.1);
    EXPECT_TRUE(p.has_via);
    EXPECT_NEAR(p.via.x, 5.0, 1e-12);
    EXPECT_NEAR(p.via.y, 0.0, 1e-12);   // 開口部の中心 = ロボットの直線経路上
    EXPECT_TRUE(p.pos.x > 5.0 && p.pos.x <= 22.0);
    for (Vec2 q : {p.pos, p.via, p.goal}) {
      EXPECT_TRUE(std::fabs(q.y) < 1.75 && q.x > -6.0 && q.x < 24.0);
      EXPECT_TRUE(distanceToWalls(q, s.walls) >= 0.35);
    }
    for (const Segment& wl : s.walls) {
      EXPECT_TRUE(!segmentsIntersect(p.pos, p.via, wl.a, wl.b));
      EXPECT_TRUE(!segmentsIntersect(p.via, p.goal, wl.a, wl.b));
    }
    // 開口部の中心から壁まで衝突判定(0.3 + 0.05)より遠い
    EXPECT_TRUE(0.5 * w > 0.35);
  }
}

TEST(indoor_side_door_definition) {
  int branch_count[3] = {0, 0, 0};
  for (unsigned seed = 1; seed <= 20; ++seed) {
    std::mt19937 rng(seed);
    const Scenario s = makeScenario("side_door", rng);
    EXPECT_TRUE(s.peds.size() == 1);
    const PedInit& p = s.peds[0];
    EXPECT_TRUE(p.v0 >= 0.5 && p.v0 <= 2.1);
    // 開口部の幅: 上側の枝通路の 2 本の縦壁の間隔
    double xl = 1e9, xr = -1e9;
    for (const Segment& wl : s.walls)
      if (wl.a.x == wl.b.x && std::min(wl.a.y, wl.b.y) > 0) { xl = std::min(xl, wl.a.x); xr = std::max(xr, wl.a.x); }
    const double w = xr - xl;
    EXPECT_TRUE(w >= 1.1 - 1e-9 && w <= 1.4 + 1e-9);   // 配置の調整後(0.9〜1.2 では sfm2 の人が角の壁の力で通れない)
    EXPECT_NEAR(0.5 * (xl + xr), 3.0, 1e-12);
    // 下側にも同じ幅
    double xl2 = 1e9, xr2 = -1e9;
    for (const Segment& wl : s.walls)
      if (wl.a.x == wl.b.x && std::max(wl.a.y, wl.b.y) < 0) { xl2 = std::min(xl2, wl.a.x); xr2 = std::max(xr2, wl.a.x); }
    EXPECT_NEAR(xr2 - xl2, w, 1e-12);
    std::vector<Vec2> pts{p.pos, p.goal};
    if (p.has_via) pts.push_back(p.via);
    for (Vec2 q : pts) {
      EXPECT_TRUE(insideSideDoor(q, w));
      EXPECT_TRUE(distanceToWalls(q, s.walls) >= 0.35);
    }
    EXPECT_TRUE(std::fabs(p.pos.y) >= 1.75 && std::fabs(p.pos.y) <= 8.5);
    std::vector<Vec2> path{p.pos};
    if (p.has_via) path.push_back(p.via);
    path.push_back(p.goal);
    for (std::size_t k = 0; k + 1 < path.size(); ++k)
      for (const Segment& wl : s.walls) EXPECT_TRUE(!segmentsIntersect(path[k], path[k + 1], wl.a, wl.b));
    if (!p.has_via) ++branch_count[0];
    else if (p.goal.x < 0) ++branch_count[1];
    else ++branch_count[2];
  }
  EXPECT_TRUE(branch_count[0] + branch_count[1] + branch_count[2] == 20);
}

TEST(indoor_group_oncoming_definition) {
  for (unsigned seed = 1; seed <= 20; ++seed) {
    std::mt19937 rng(seed);
    const Scenario s = makeScenario("group_oncoming", rng);
    EXPECT_TRUE(s.peds.size() == 2 || s.peds.size() == 3);
    for (const PedInit& p : s.peds) {
      EXPECT_TRUE(p.group_id == 100 && p.group_id == s.peds[0].group_id);
      EXPECT_TRUE(p.v0 == s.peds[0].v0);
      EXPECT_TRUE(p.v0 >= 0.5 && p.v0 <= 2.1);
      EXPECT_TRUE(p.kind == 0 && !p.has_via && !p.frozen && !p.loop);
      EXPECT_TRUE(distanceToWalls(p.pos, s.walls) >= 0.35);
      EXPECT_TRUE(p.pos.x > 12.0 && p.pos.x < 16.0);
      EXPECT_NEAR(p.goal.y, p.pos.y, 1e-12);
    }
    for (std::size_t i = 0; i + 1 < s.peds.size(); ++i) {
      const double gap = s.peds[i + 1].pos.y - s.peds[i].pos.y;
      EXPECT_TRUE(gap >= 0.65 - 1e-9 && gap <= 0.85 + 1e-9);
    }
  }
}

TEST(indoor_scenarios_reproducible) {
  for (const std::string& n : indoorScenarioNames()) {
    std::mt19937 a(7), b(7);
    const Scenario x = makeScenario(n, a), y = makeScenario(n, b);
    EXPECT_TRUE(x.peds.size() == y.peds.size() && x.walls.size() == y.walls.size());
    for (std::size_t i = 0; i < x.peds.size(); ++i) {
      EXPECT_TRUE(x.peds[i].pos.x == y.peds[i].pos.x && x.peds[i].pos.y == y.peds[i].pos.y);
      EXPECT_TRUE(x.peds[i].goal.x == y.peds[i].goal.x && x.peds[i].goal.y == y.peds[i].goal.y);
      EXPECT_TRUE(x.peds[i].v0 == y.peds[i].v0 && x.peds[i].has_via == y.peds[i].has_via);
    }
  }
}

// 配置の検査(制御器なし。ロボットを入れない PedModel、robot = nullptr、シード 1–20)
TEST(indoor_placement_peds_alone) {
  for (const std::string& n : indoorScenarioNames())
    for (unsigned seed = 1; seed <= 20; ++seed) {
      std::mt19937 rng(seed);
      const Scenario s = makeScenario(n, rng);
      std::vector<Pedestrian> peds = makePeds(s);
      PedModel model(PedParams{}, s.walls, seed + 4242u);
      std::vector<char> done(peds.size(), 0);
      const double dt = 0.02;
      std::vector<double> reach(peds.size(), -1.0);
      bool group_ok = true;
      for (int k = 0; k < static_cast<int>(40.0 / dt); ++k) {
        for (std::size_t i = 0; i < peds.size(); ++i) {
          bool d = done[i] != 0;
          switchVia(s.peds[i], peds[i], d);
          done[i] = d;
        }
        model.step(peds, nullptr, dt);
        const double t = (k + 1) * dt;
        for (std::size_t i = 0; i < peds.size(); ++i)
          if (reach[i] < 0 && std::hypot(peds[i].pos.x - s.peds[i].goal.x, peds[i].pos.y - s.peds[i].goal.y) < 0.5) reach[i] = t;
        if (n == "group_oncoming" && k + 1 == static_cast<int>(6.0 / dt)) {
          std::vector<double> ys;
          double xmin = 1e9, xmax = -1e9;
          for (const Pedestrian& p : peds) { ys.push_back(p.pos.y); xmin = std::min(xmin, p.pos.x); xmax = std::max(xmax, p.pos.x); }
          std::sort(ys.begin(), ys.end());
          for (std::size_t i = 0; i + 1 < ys.size(); ++i) group_ok = group_ok && ys[i + 1] - ys[i] >= 0.4 && ys[i + 1] - ys[i] <= 1.3;
          group_ok = group_ok && xmax - xmin <= 0.8;
        }
      }
      if (n == "group_oncoming") {
        EXPECT_TRUE(group_ok);   // t=6 s の隊形(間隔 0.4〜1.3 m、x の広がり <= 0.8 m)
      } else {
        for (double r : reach) EXPECT_TRUE(r > 0.0);   // 40 s 以内に目標の 0.5 m 以内へ着く
      }
    }
}

// 見えない人。blind は別の一覧。all / rear / indoor に入らない
TEST(blind_scenarios_separate_list) {
  const std::vector<std::string> expected{"blind_corner"};
  EXPECT_TRUE(blindScenarioNames() == expected);
  for (const std::string& n : blindScenarioNames()) {
    for (const std::string& m : scenarioNames()) EXPECT_TRUE(n != m);
    for (const std::string& m : rearScenarioNames()) EXPECT_TRUE(n != m);
    for (const std::string& m : indoorScenarioNames()) EXPECT_TRUE(n != m);
  }
}

TEST(blind_corner_definition) {
  for (unsigned seed = 1; seed <= 200; ++seed) {
    std::mt19937 rng(seed);
    const Scenario s = makeScenario("blind_corner", rng);
    EXPECT_TRUE(s.peds.size() == 1);
    EXPECT_TRUE(s.opening_w >= 1.2 && s.opening_w <= 1.6);
    EXPECT_TRUE(s.walls.size() == 10);
    const PedInit& p = s.peds[0];
    EXPECT_TRUE(p.pos.x == 4.0 && std::abs(p.pos.y) >= 1.0 && std::abs(p.pos.y) <= 11.5);
    EXPECT_TRUE(p.v0 > 0.0);
    if (p.has_via) EXPECT_TRUE(p.via.x == 4.0 && std::abs(p.via.y) == 0.4 && p.via.y * p.pos.y > 0.0);
  }
}

TEST(blind_corner_reproducible) {
  std::mt19937 a(7), b(7);
  const Scenario x = makeScenario("blind_corner", a), y = makeScenario("blind_corner", b);
  EXPECT_TRUE(x.opening_w == y.opening_w && x.peds[0].pos.y == y.peds[0].pos.y && x.peds[0].goal.y == y.peds[0].goal.y);
  EXPECT_TRUE(x.peds[0].has_via == y.peds[0].has_via && x.peds[0].v0 == y.peds[0].v0);
}

// 配置の検査(シード 1–20): 人単独で 40 s 以内に目標へ着く / t = 0 でロボットの開始位置から人が見えない
TEST(blind_placement_peds_alone) {
  for (unsigned seed = 1; seed <= 20; ++seed) {
    std::mt19937 rng(seed);
    const Scenario s = makeScenario("blind_corner", rng);
    EXPECT_TRUE(!lineOfSight({s.start.x, s.start.y}, s.peds[0].pos, s.walls));
    std::vector<Pedestrian> peds = makePeds(s);
    PedModel model(PedParams{}, s.walls, seed + 4242u);
    std::vector<char> done(peds.size(), 0);
    const double dt = 0.02;
    double reach = -1.0;
    for (int k = 0; k < static_cast<int>(40.0 / dt); ++k) {
      bool d = done[0] != 0;
      switchVia(s.peds[0], peds[0], d);
      done[0] = d;
      model.step(peds, nullptr, dt);
      if (reach < 0 && std::hypot(peds[0].pos.x - s.peds[0].goal.x, peds[0].pos.y - s.peds[0].goal.y) < 0.5) reach = (k + 1) * dt;
    }
    EXPECT_TRUE(reach > 0.0);
  }
}

// 急に止まる・振り返る人。sudden は別の一覧。all / rear / indoor / blind に入らない
TEST(sudden_scenarios_separate_list) {
  const std::vector<std::string> expected{"sudden_stop", "sudden_turn", "sudden_reverse"};
  EXPECT_TRUE(suddenScenarioNames() == expected);
  for (const std::string& n : suddenScenarioNames()) {
    for (const std::string& m : scenarioNames()) EXPECT_TRUE(n != m);
    for (const std::string& m : rearScenarioNames()) EXPECT_TRUE(n != m);
    for (const std::string& m : indoorScenarioNames()) EXPECT_TRUE(n != m);
    for (const std::string& m : blindScenarioNames()) EXPECT_TRUE(n != m);
  }
}

TEST(sudden_definition_and_reproducible) {
  for (unsigned seed = 1; seed <= 200; ++seed) {
    Scenario first;
    for (int k = 0; k < 3; ++k) {
      std::mt19937 rng(seed);
      const Scenario s = makeScenario(suddenScenarioNames()[k], rng);
      EXPECT_TRUE(s.peds.size() == 1);
      const PedInit& p = s.peds[0];
      EXPECT_TRUE(p.event.kind == k + 1);
      EXPECT_TRUE(p.pos.x >= 2.7 - 1e-9 && p.pos.x <= 3.3 + 1e-9 && std::abs(p.pos.y) <= 0.3 + 1e-9);
      EXPECT_TRUE(p.goal.x == 28.0 && p.goal.y == p.pos.y && p.v0 > 0.0 && !p.has_via);
      EXPECT_TRUE(p.event.trigger_d >= 2.5 && p.event.trigger_d <= 2.65 && p.event.trigger_d < p.pos.x && p.event.t_max == 12.0);
      if (k == 0) EXPECT_TRUE(p.event.pause >= 2.0 && p.event.pause <= 4.0);
      else EXPECT_TRUE(p.event.pause == 0.5);
      EXPECT_TRUE(std::abs(p.event.turn_y) == 1.2 && p.event.reverse_x == -4.0);
      if (k == 0) first = s;
      else {   // 同シードなら 3 本は同配置(事象の種類だけ違う)
        EXPECT_TRUE(p.pos.x == first.peds[0].pos.x && p.pos.y == first.peds[0].pos.y && p.v0 == first.peds[0].v0);
        EXPECT_TRUE(p.event.trigger_d == first.peds[0].event.trigger_d && p.event.turn_y == first.peds[0].event.turn_y);
      }
      std::mt19937 rng2(seed);
      const Scenario s2 = makeScenario(suddenScenarioNames()[k], rng2);
      EXPECT_TRUE(s2.peds[0].pos.x == p.pos.x && s2.peds[0].event.pause == p.event.pause);
    }
  }
}

// 事象なし(kind 0、既存シナリオ)なら goal / v0 に触れない
TEST(event_off_leaves_goal_and_v0) {
  for (const std::string& n : scenarioNames())
    for (unsigned seed = 1; seed <= 5; ++seed) {
      std::mt19937 rng(seed);
      const Scenario s = makeScenario(n, rng);
      for (const PedInit& pi : s.peds) EXPECT_TRUE(pi.event.kind == 0);
    }
  Pedestrian p;
  p.pos = {1.0, 0.0}; p.goal = {5.0, 0.0}; p.v0 = 1.3;
  PedEventSpec spec;   // kind 0
  PedEventState st;
  for (int k = 0; k < 1000; ++k) stepPedEvent(spec, st, p, {0.0, 0.0}, k * 0.02, 0.02);
  EXPECT_TRUE(p.goal.x == 5.0 && p.goal.y == 0.0 && p.v0 == 1.3 && st.phase == 0);
}

// turn / reverse の再開先は発火位置(x_e, y_e)から決まる(再開時の位置ではない)
TEST(event_resume_goal_uses_fire_position) {
  for (int kind = 2; kind <= 3; ++kind) {
    PedEventSpec spec;
    spec.kind = kind; spec.trigger_d = 2.0; spec.pause = 0.5; spec.turn_y = -1.2; spec.reverse_x = -4.0;
    Pedestrian p;
    p.pos = {5.0, 0.2}; p.goal = {28.0, 0.2}; p.v0 = 0.65;
    PedEventState st;
    const double dt = 0.02;
    for (int k = 0; k < 400 && st.phase != 3; ++k) {
      stepPedEvent(spec, st, p, {4.0, 0.0}, k * dt, dt);   // 最初の呼び出しで発火(5.0 - 4.0 <= 2.0)
      if (st.phase == 1) p.pos.x += 0.01;                  // 減速中に動く(発火位置と再開時の位置が違うようにする)
    }
    EXPECT_TRUE(st.phase == 3 && st.pos_fire.x == 5.0 && st.pos_fire.y == 0.2);
    if (kind == 2) EXPECT_TRUE(p.goal.x == 5.0 && p.goal.y == -1.2);
    else EXPECT_TRUE(p.goal.x == -4.0 && p.goal.y == 0.2);
  }
}

// 配置の検査(制御器なし。ロボットを入れない 4 モデル、シード 1–20)。ロボットは遠くに置き、時刻 t_max = 12 s で発火させる。
// 減速 → 停止 → 再開(stop は速さが戻る、turn は y = turn_y 付近へ着く、reverse は発火位置より 2 m 以上後ろへ)を 40 s 以内に満たす
TEST(sudden_placement_peds_alone) {
  const char* models[] = {"legacy", "sfm2", "ttc", "orca"};
  for (const char* mname : models)
    for (int k = 0; k < 3; ++k)
      for (unsigned seed = 1; seed <= 20; ++seed) {
        std::mt19937 rng(seed);
        const Scenario s = makeScenario(suddenScenarioNames()[k], rng);
        std::vector<Pedestrian> peds = makePeds(s);
        const std::string m = mname;
        const SocialForceWorld legacy(SfmParams{}, s.walls);
        PedModel sfm2(PedParams{}, s.walls, seed + 4242u);
        TtcPedModel ttc(TtcParams{}, s.walls, seed + 4242u);
        const OrcaPedModel orca(OrcaParams{}, s.walls);
        PedEventState st;
        const double dt = 0.02;
        bool stopped = false, resumed = false, turned = false, backed = false;
        double x_fire = 0.0;
        for (int i = 0; i < static_cast<int>(40.0 / dt); ++i) {
          const double t = i * dt;
          stepPedEvent(s.peds[0].event, st, peds[0], {-100.0, 0.0}, t, dt);
          if (st.phase == 1 && st.t_fire == t) x_fire = peds[0].pos.x;
          if (m == "legacy") legacy.step(peds, nullptr, dt);
          else if (m == "sfm2") sfm2.step(peds, nullptr, dt);
          else if (m == "ttc") ttc.step(peds, nullptr, dt);
          else orca.step(peds, nullptr, dt);
          const double sp = std::hypot(peds[0].vel.x, peds[0].vel.y);
          if (st.phase >= 1 && sp < 0.1) stopped = true;
          if (stopped && st.phase == 3) {
            if (sp >= 0.3) resumed = true;
            if (std::abs(peds[0].pos.y - s.peds[0].event.turn_y) < 0.3) turned = true;
            if (peds[0].pos.x <= x_fire - 2.0) backed = true;
          }
        }
        EXPECT_TRUE(st.phase == 3 && st.t_fire == 12.0 && stopped);
        if (k == 0) EXPECT_TRUE(resumed);
        if (k == 1) EXPECT_TRUE(turned);
        if (k == 2) EXPECT_TRUE(backed);
      }
}

// エレベーター前・出口の流れ(候補 6)。elevator は別の一覧。all / rear / indoor / blind / sudden に入らない
TEST(elevator_scenarios_separate_list) {
  const std::vector<std::string> expected{"elevator_flow"};
  EXPECT_TRUE(elevatorScenarioNames() == expected);
  for (const std::string& n : elevatorScenarioNames()) {
    for (const std::string& m : scenarioNames()) EXPECT_TRUE(n != m);
    for (const std::string& m : rearScenarioNames()) EXPECT_TRUE(n != m);
    for (const std::string& m : indoorScenarioNames()) EXPECT_TRUE(n != m);
    for (const std::string& m : blindScenarioNames()) EXPECT_TRUE(n != m);
    for (const std::string& m : suddenScenarioNames()) EXPECT_TRUE(n != m);
  }
}

// 定義と配置の検査のうち開始時の部分(人数 4〜6 + 4〜6、開始時に重ならない、壁・ロボットから離れている)。200 シード
TEST(elevator_definition_and_start_placement) {
  int seen_out[7] = {0}, seen_wait[7] = {0};
  for (unsigned seed = 1; seed <= 200; ++seed) {
    std::mt19937 rng(seed);
    const Scenario s = makeScenario("elevator_flow", rng);
    int n_out = 0, n_wait = 0;
    for (const PedInit& p : s.peds) (p.frozen ? n_wait : n_out)++;
    EXPECT_TRUE(n_out >= 4 && n_out <= 6 && n_wait >= 4 && n_wait <= 6);
    ++seen_out[std::min(n_out, 6)]; ++seen_wait[std::min(n_wait, 6)];
    for (std::size_t i = 0; i < s.peds.size(); ++i) {
      const PedInit& p = s.peds[i];
      EXPECT_TRUE(p.pos.x > -3.6 && p.pos.x < 17.6 && std::fabs(p.pos.y) < 4.1);        // 壁から 0.4 m 以上
      EXPECT_TRUE(std::hypot(p.pos.x - s.start.x, p.pos.y - s.start.y) >= 5.0);          // ロボットから離れている
      for (std::size_t j = i + 1; j < s.peds.size(); ++j)
        EXPECT_TRUE(std::hypot(p.pos.x - s.peds[j].pos.x, p.pos.y - s.peds[j].pos.y) >= 0.6);   // 重ならない
      if (p.frozen) EXPECT_TRUE(p.pos.x >= 10.5 && p.pos.x <= 13.9 && std::fabs(p.pos.y) >= 1.5 && std::fabs(p.pos.y) <= 4.1);
      if (p.frozen) EXPECT_TRUE(p.v0 == 0.0 && p.goal.x == p.pos.x && p.goal.y == p.pos.y && !p.has_via);
      else EXPECT_TRUE(p.pos.x >= 15.7 && p.goal.x == -3.0 && p.v0 >= 0.5 && !p.has_via && p.event.kind == 0);
    }
    {   // (d) 待っている人の塊: 扉(x = 18)に近い人までの距離 <= 5.5 m、x の広がり <= 3.0 m、ロボットの目標 (12,0) から 3.0 m 以内に 2 人以上
      double xmin = 1e9, xmax = -1e9;
      int near_goal = 0;
      for (const PedInit& p : s.peds) if (p.frozen) { xmin = std::min(xmin, p.pos.x); xmax = std::max(xmax, p.pos.x); if (std::hypot(p.pos.x - 12.0, p.pos.y) <= 3.0) ++near_goal; }
      EXPECT_TRUE(18.0 - xmax <= 5.5 && xmax - xmin <= 3.0 && near_goal >= 2);
    }
    std::mt19937 rng2(seed);
    const Scenario s2 = makeScenario("elevator_flow", rng2);   // 再現
    EXPECT_TRUE(s2.peds.size() == s.peds.size());
    for (std::size_t i = 0; i < s.peds.size(); ++i) EXPECT_TRUE(s2.peds[i].pos.x == s.peds[i].pos.x && s2.peds[i].pos.y == s.peds[i].pos.y && s2.peds[i].v0 == s.peds[i].v0);
  }
  for (int n = 4; n <= 6; ++n) EXPECT_TRUE(seen_out[n] > 0 && seen_wait[n] > 0);
}

// 配置の検査(制御器なし。ロボットを入れない 4 モデル、シード 1–20)。ロボットは名目の経路 x(t) = clamp((t-0.5)*0.8, 0, 12)、y = 0 を動くとみなす。
// (a) 出てくる人が t_r(release_t)から 40 s 以内に全員 目標の 0.5 m 以内へ着く、(b) ロボットと出会う: 名目のロボットとの最小距離 <= 1.5 m で、その時刻が 3〜30 s、
// (c) 開始直後に起きない: t < 3 s の間、全員が名目のロボットから 3 m より遠い、(e) 出てくる人が待っている人の 2.0 m 以内を通る
TEST(elevator_placement_peds_alone) {
  const char* models[] = {"legacy", "sfm2", "ttc", "orca"};
  for (const char* mname : models)
    for (unsigned seed = 1; seed <= 20; ++seed) {
      std::mt19937 rng(seed);
      const Scenario s = makeScenario("elevator_flow", rng);
      std::vector<Pedestrian> peds = makePeds(s);
      const std::string m = mname;
      const SocialForceWorld legacy(SfmParams{}, s.walls);
      PedModel sfm2(PedParams{}, s.walls, seed + 4242u);
      TtcPedModel ttc(TtcParams{}, s.walls, seed + 4242u);
      const OrcaPedModel orca(OrcaParams{}, s.walls);
      const double dt = 0.02;
      std::vector<double> reach(peds.size(), -1.0);
      double dmin = 1e9, t_dmin = -1.0, dmin_early = 1e9, dmin_wait = 1e9;
      const double t_r = s.peds.front().release_t;   // 出てくる人が先頭
      for (int k = 0; k < static_cast<int>((t_r + 40.0) / dt); ++k) {
        const double t = k * dt;
        for (std::size_t i = 0; i < peds.size(); ++i)   // release_t の解除(main.cpp と同じ)
          if (s.peds[i].release_t > 0.0 && peds[i].frozen && t >= s.peds[i].release_t) {
            peds[i].frozen = false;
            const double gd = std::hypot(s.peds[i].goal.x - peds[i].pos.x, s.peds[i].goal.y - peds[i].pos.y);
            peds[i].vel = {s.peds[i].v0 * (s.peds[i].goal.x - peds[i].pos.x) / gd, s.peds[i].v0 * (s.peds[i].goal.y - peds[i].pos.y) / gd};
          }
        const double rx = std::min(12.0, std::max(0.0, (t - 0.5) * 0.8));
        for (std::size_t i = 0; i < peds.size(); ++i) {
          const double d = std::hypot(peds[i].pos.x - rx, peds[i].pos.y);
          if (d < dmin) { dmin = d; t_dmin = t; }
          if (t < 3.0) dmin_early = std::min(dmin_early, d);
          if (!s.peds[i].frozen) for (std::size_t j = 0; j < peds.size(); ++j) if (s.peds[j].frozen) dmin_wait = std::min(dmin_wait, std::hypot(peds[i].pos.x - peds[j].pos.x, peds[i].pos.y - peds[j].pos.y));
        }
        if (m == "legacy") legacy.step(peds, nullptr, dt);
        else if (m == "sfm2") sfm2.step(peds, nullptr, dt);
        else if (m == "ttc") ttc.step(peds, nullptr, dt);
        else orca.step(peds, nullptr, dt);
        for (std::size_t i = 0; i < peds.size(); ++i)
          if (!s.peds[i].frozen && reach[i] < 0 && std::hypot(peds[i].pos.x - s.peds[i].goal.x, peds[i].pos.y - s.peds[i].goal.y) < 0.5) reach[i] = (k + 1) * dt - t_r;
      }
      for (std::size_t i = 0; i < peds.size(); ++i) if (!s.peds[i].frozen) EXPECT_TRUE(reach[i] > 0.0);
      EXPECT_TRUE(dmin <= 1.5 && t_dmin >= 3.0 && t_dmin <= 30.0);
      EXPECT_TRUE(dmin_early > 3.0);
      EXPECT_TRUE(dmin_wait <= 2.0);   // (e) 出てくる流れが待っている人の塊の 2.0 m 以内を通る
    }
}

// release_t(候補 6): (i) elevator 以外は全員 0、(ii) 出てくる人は全員同じ t_r ∈ [9, 11]、待っている人は 0、(iii) t < t_r の間、出てくる人は動かない
TEST(elevator_release_t) {
  std::vector<std::string> others;
  for (const auto* v : {&scenarioNames(), &rearScenarioNames(), &indoorScenarioNames(), &blindScenarioNames(), &suddenScenarioNames()})
    others.insert(others.end(), v->begin(), v->end());
  for (const std::string& n : others)
    for (unsigned seed = 1; seed <= 20; ++seed) {
      std::mt19937 rng(seed);
      const Scenario s = makeScenario(n, rng);
      for (const PedInit& p : s.peds) EXPECT_TRUE(p.release_t == 0.0);
    }
  for (unsigned seed = 1; seed <= 20; ++seed) {
    std::mt19937 rng(seed);
    const Scenario s = makeScenario("elevator_flow", rng);
    double t_r = -1.0;
    for (const PedInit& p : s.peds) {
      if (p.frozen) { EXPECT_TRUE(p.release_t == 0.0); continue; }
      if (t_r < 0.0) t_r = p.release_t;
      EXPECT_TRUE(p.release_t == t_r && t_r >= 9.0 && t_r <= 11.0);
    }
    std::vector<Pedestrian> peds = makePeds(s);
    PedModel sfm2(PedParams{}, s.walls, seed + 4242u);
    const double dt = 0.02;
    for (int k = 0; k < static_cast<int>((t_r - 0.1) / dt); ++k) sfm2.step(peds, nullptr, dt);
    for (std::size_t i = 0; i < peds.size(); ++i)
      EXPECT_TRUE(peds[i].pos.x == s.peds[i].pos.x && peds[i].pos.y == s.peds[i].pos.y);
  }
}

MINI_TEST_MAIN()
