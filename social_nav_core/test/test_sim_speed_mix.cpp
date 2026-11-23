// 速さの違う人(候補 8)の単体テスト: 付与(B)、周期停止の状態機械、制御器なし・ロボットなしの走行(C。sfm2、シード 1–20)。
// 21–120 の配置は生成しない(200 シードは 1000 番台から)。
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>

#include "mini_test.hpp"
#include "sim/ped_model.hpp"
#include "sim/ped_pause.hpp"
#include "sim/ped_speed_mix.hpp"
#include "sim/ped_variants.hpp"
#include "sim/scenarios.hpp"

using namespace social_nav;
using namespace social_nav::sim;

namespace {
const char* kTargets[] = {"corridor_oncoming", "crossing", "crowd"};
const char* kNonTargets[] = {"overtake", "follow", "static_group"};

// main.cpp と同じ手順で作る(makeScenario → applyPedVariants(seed+4243)→ applySpeedMix(seed+8647))
Scenario build(const char* name, unsigned seed, bool mix) {
  std::mt19937 rng(seed);
  Scenario s = makeScenario(name, rng);
  applyPedVariants(s, seed + 4243u);
  if (mix) applySpeedMix(s, seed + 8647u);
  return s;
}

std::vector<Pedestrian> toPeds(const Scenario& s) {
  std::vector<Pedestrian> peds;
  for (std::size_t i = 0; i < s.peds.size(); ++i) {
    const PedInit& pi = s.peds[i];
    Pedestrian p;
    p.id = static_cast<uint32_t>(i + 1);
    p.pos = pi.pos; p.home = pi.pos; p.goal = pi.goal; p.v0 = pi.v0; p.radius = 0.3;
    p.frozen = pi.frozen; p.loop = pi.loop; p.kind = pi.kind; p.group_id = pi.group_id;
    const double d = std::hypot(pi.goal.x - pi.pos.x, pi.goal.y - pi.pos.y);
    if (!pi.frozen && d > 1e-9) p.vel = {pi.v0 * (pi.goal.x - pi.pos.x) / d, pi.v0 * (pi.goal.y - pi.pos.y) / d};
    peds.push_back(p);
  }
  return peds;
}

struct PedStat {
  int type{0};
  double t_win{0.0};        // 窓の長さ [s](出発から目標の 0.5 m 以内に着くまで。loop の人と着かない人は kWindow)
  double t_slow{0.0};       // 窓のうち速さ < 0.1 m/s の時間
  double sum_moving{0.0}, t_moving{0.0};   // 速さ >= 0.1 の間の積算
  double t_first_stop{-1.0};
  double moving_mean() const { return t_moving > 0.0 ? sum_moving / t_moving : 0.0; }
};
constexpr double kWindow = 60.0;       // C の統計の窓
constexpr double kStopWindow = 120.0;  // D の全員停止の検査の窓(判定の値 60 s より長くする。統計は kWindow までで打ち切る)

// 制御器なし・ロボットなし。main.cpp と同じ順(周期停止 → 歩行者の step)
// all_stopped_max = 目標に着いていない人が全員 速さ < 0.1 m/s のまま続いた最長の時間(D の「全員が止まる走行」の確認)
std::vector<PedStat> runAlone(const Scenario& s, unsigned seed, double* all_stopped_max = nullptr) {
  std::vector<Pedestrian> peds = toPeds(s);
  PedModel model(PedParams{}, s.walls, seed + 4242u);
  std::vector<PedPauseState> ps(peds.size());
  std::vector<PedStat> st(peds.size());
  std::vector<char> done(peds.size(), 0);
  const double dt = 0.02;
  double run_stopped = 0.0, max_stopped = 0.0;
  for (std::size_t i = 0; i < peds.size(); ++i) st[i].type = s.peds[i].speed_type;
  for (int k = 0; k < static_cast<int>(kStopWindow / dt); ++k) {
    const double t = (k + 1) * dt;
    for (std::size_t i = 0; i < peds.size(); ++i)
      if (s.peds[i].pause.period > 0.0) stepPedPause(s.peds[i].pause, ps[i], peds[i], t, dt);
    model.step(peds, nullptr, dt);
    bool any_active = false, all_slow = true;
    for (std::size_t i = 0; i < peds.size(); ++i)
      if (!done[i] && !s.peds[i].frozen) { any_active = true; if (std::hypot(peds[i].vel.x, peds[i].vel.y) >= 0.1) all_slow = false; }
    run_stopped = (any_active && all_slow) ? run_stopped + dt : 0.0;
    max_stopped = std::max(max_stopped, run_stopped);
    if (t > kWindow + 1e-9) continue;   // 統計は 60 s まで
    for (std::size_t i = 0; i < peds.size(); ++i) {
      if (done[i]) continue;
      const double sp = std::hypot(peds[i].vel.x, peds[i].vel.y);
      st[i].t_win += dt;
      if (sp < 0.1) st[i].t_slow += dt; else { st[i].sum_moving += sp * dt; st[i].t_moving += dt; }
      if (!s.peds[i].loop && std::hypot(peds[i].pos.x - s.peds[i].goal.x, peds[i].pos.y - s.peds[i].goal.y) < 0.5) done[i] = 1;
    }
  }
  for (std::size_t i = 0; i < peds.size(); ++i) st[i].t_first_stop = ps[i].t_first_stop;
  if (all_stopped_max) *all_stopped_max = max_stopped;
  return st;
}
}  // namespace

// B: 付与の決定性、配置・goal・人数は off と同じ、対象外は不変、範囲、4 種別
TEST(speed_mix_assignment) {
  int seen[5] = {0}, seen_crowd[5] = {0};
  for (unsigned seed = 1001; seed <= 1200; ++seed) {
    for (const char* name : kTargets) {
      const Scenario off = build(name, seed, false), on = build(name, seed, true), on2 = build(name, seed, true);
      EXPECT_TRUE(on.peds.size() == off.peds.size() && on2.peds.size() == on.peds.size());
      for (std::size_t i = 0; i < on.peds.size(); ++i) {
        const PedInit &a = off.peds[i], &b = on.peds[i];
        EXPECT_TRUE(a.pos.x == b.pos.x && a.pos.y == b.pos.y && a.goal.x == b.goal.x && a.goal.y == b.goal.y);
        EXPECT_TRUE(a.kind == b.kind && a.group_id == b.group_id && a.frozen == b.frozen && a.loop == b.loop);
        EXPECT_TRUE(b.speed_type == on2.peds[i].speed_type && b.v0 == on2.peds[i].v0 && b.pause.period == on2.peds[i].pause.period &&
                    b.pause.pause == on2.peds[i].pause.pause && b.pause.phase == on2.peds[i].pause.phase);
        const bool target = !a.frozen && a.kind == 0 && a.group_id < 0;
        if (!target) { EXPECT_TRUE(b.speed_type == 0 && b.v0 == a.v0 && b.pause.period == 0.0); continue; }
        EXPECT_TRUE(b.speed_type >= kSpeedSlow && b.speed_type <= kSpeedAsIs);
        ++seen[b.speed_type];
        if (std::string(name) == "crowd") ++seen_crowd[b.speed_type];
        if (b.speed_type == kSpeedSlow) EXPECT_TRUE(b.v0 >= 0.5 && b.v0 <= 0.7);
        else if (b.speed_type == kSpeedHurry) EXPECT_TRUE(b.v0 >= 1.8 && b.v0 <= 2.0);
        else EXPECT_TRUE(b.v0 == a.v0);
        if (b.speed_type == kSpeedPausy) {
          EXPECT_TRUE(b.pause.period >= 4.0 && b.pause.period <= 8.0 && b.pause.pause >= 1.5 && b.pause.pause <= 3.0);
          EXPECT_TRUE(b.pause.phase >= 0.0 && b.pause.phase <= b.pause.period);
        } else {
          EXPECT_TRUE(b.pause.period == 0.0);
        }
      }
    }
    for (const char* name : kNonTargets) {   // 対象外のシナリオは on でも何も変わらない
      const Scenario off = build(name, seed, false), on = build(name, seed, true);
      for (std::size_t i = 0; i < on.peds.size(); ++i)
        EXPECT_TRUE(on.peds[i].v0 == off.peds[i].v0 && on.peds[i].speed_type == 0 && on.peds[i].pause.period == 0.0);
    }
  }
  for (int t = kSpeedSlow; t <= kSpeedAsIs; ++t) EXPECT_TRUE(seen[t] > 0 && seen_crowd[t] > 0);
  std::printf("  crowd 1 走行あたりの人数(200 シード平均): slow %.2f hurry %.2f pausy %.2f as-is %.2f\n", seen_crowd[kSpeedSlow] / 200.0,
              seen_crowd[kSpeedHurry] / 200.0, seen_crowd[kSpeedPausy] / 200.0, seen_crowd[kSpeedAsIs] / 200.0);
}

// 周期停止の状態機械: 止まる・再開・goal と v0 が元に戻る(goal には触れない)、t < 3 s に止まり始めない、period 0 は何もしない
TEST(ped_pause_state_machine) {
  PedPauseSpec spec;
  spec.period = 5.0; spec.pause = 2.0; spec.phase = 0.5;   // phase < 3 s でも 3 s まで止まり始めない
  Pedestrian p;
  p.pos = {0.0, 0.0}; p.goal = {10.0, 2.0}; p.v0 = 1.2;
  const Vec2 goal0 = p.goal;
  PedPauseState st;
  const double dt = 0.02;
  double t_stop_begin = -1.0, t_resume = -1.0;
  bool v0_floor = false;
  for (int k = 0; k < 2000; ++k) {
    const double t = (k + 1) * dt;
    stepPedPause(spec, st, p, t, dt);
    EXPECT_TRUE(p.goal.x == goal0.x && p.goal.y == goal0.y);   // goal に触れない
    if (st.phase == 1 && t_stop_begin < 0.0) t_stop_begin = t;
    if (st.phase == 2 && p.v0 <= kPauseV0Floor + 1e-12) v0_floor = true;
    if (t_stop_begin > 0.0 && st.phase == 0 && t_resume < 0.0) { t_resume = t; EXPECT_TRUE(p.v0 == 1.2); }
  }
  EXPECT_TRUE(t_stop_begin >= 3.0 && t_stop_begin < 3.1);
  EXPECT_TRUE(v0_floor && t_resume > t_stop_begin + 1.2 / 0.8 + 2.0 - 0.1 && t_resume < t_stop_begin + 1.2 / 0.8 + 2.0 + 0.1);
  EXPECT_TRUE(st.n_stops >= 3);   // 40 s で繰り返し止まる
  Pedestrian q; q.v0 = 1.1; PedPauseState s2; PedPauseSpec off;
  for (int k = 0; k < 500; ++k) stepPedPause(off, s2, q, k * dt, dt);
  EXPECT_TRUE(q.v0 == 1.1 && s2.phase == 0 && s2.n_stops == 0);
}

// loop の人(crowd)に周期停止を付けても goal の周回が壊れない: 停止中に goal を差し替えない、再開後に周回が続く
TEST(ped_pause_loop_goal_survives) {
  int n_swaps = 0;   // pausy の人の goal の入れ替わり回数(シード 1–20 の合計)
  for (unsigned seed = 1; seed <= 20; ++seed) {
    const Scenario s = build("crowd", seed, true);
    std::vector<Pedestrian> peds = toPeds(s);
    PedModel model(PedParams{}, s.walls, seed + 4242u);
    std::vector<PedPauseState> ps(peds.size());
    const double dt = 0.02;
    std::vector<Vec2> prev_goal;
    for (const Pedestrian& p : peds) prev_goal.push_back(p.goal);
    for (int k = 0; k < static_cast<int>(60.0 / dt); ++k) {
      const double t = (k + 1) * dt;
      for (std::size_t i = 0; i < peds.size(); ++i)
        if (s.peds[i].pause.period > 0.0) stepPedPause(s.peds[i].pause, ps[i], peds[i], t, dt);
      model.step(peds, nullptr, dt);
      for (std::size_t i = 0; i < peds.size(); ++i) {
        if (s.peds[i].pause.period > 0.0 && (peds[i].goal.x != prev_goal[i].x || peds[i].goal.y != prev_goal[i].y)) ++n_swaps;
        prev_goal[i] = peds[i].goal;
      }
    }
    for (std::size_t i = 0; i < peds.size(); ++i) {
      if (s.peds[i].pause.period <= 0.0) continue;
      const Vec2 a = s.peds[i].goal, b = s.peds[i].pos;   // goal は 2 点(元の goal と home)のどちらかのまま
      const bool g_ok = (peds[i].goal.x == a.x && peds[i].goal.y == a.y) || (peds[i].goal.x == b.x && peds[i].goal.y == b.y);
      EXPECT_TRUE(g_ok && ps[i].n_stops >= 1);
    }
  }
  std::printf("  pausy の goal の入れ替わり %d 回\n", n_swaps);
  EXPECT_TRUE(n_swaps >= 1);   // 周回が実際に続いている(入れ替わりが 0 なら「再開後に周回が続く」を見ていない)
}

// C: 速さが走行に現れる(制御器なし・ロボットなし、sfm2、シード 1–20)
TEST(speed_mix_shows_in_motion) {
  int n_hurry = 0, ok_hurry = 0, n_slow = 0, ok_slow = 0;   // C1(corridor_oncoming + crossing)
  int seeds_ok = 0;                                           // C2(crowd)
  int n_pausy = 0, ok_pausy = 0;                              // C3(3 本すべて)
  double min_hurry = 1e9, max_slow = 0.0, worst_stopped = 0.0;
  double sum_c2[5] = {0}; int cnt_c2[5] = {0};
  for (unsigned seed = 1; seed <= 20; ++seed) {
    for (const char* name : kTargets) {
      const Scenario s = build(name, seed, true);
      double stopped = 0.0;
      const std::vector<PedStat> st = runAlone(s, seed, &stopped);
      worst_stopped = std::max(worst_stopped, stopped);
      for (const PedStat& r : st) {
        if (r.type == kSpeedPausy) {
          ++n_pausy;
          if (r.t_slow >= 0.10 * r.t_win) ++ok_pausy;
          EXPECT_TRUE(r.t_first_stop < 0.0 || r.t_first_stop >= 3.0);
        }
        if (std::string(name) == "crowd") continue;
        if (r.type == kSpeedHurry) { ++n_hurry; min_hurry = std::min(min_hurry, r.moving_mean()); if (r.moving_mean() >= 1.5) ++ok_hurry; }
        if (r.type == kSpeedSlow) { ++n_slow; max_slow = std::max(max_slow, r.moving_mean()); if (r.moving_mean() <= 0.8) ++ok_slow; }
      }
      if (std::string(name) == "crowd") {
        double sum[5] = {0}; int cnt[5] = {0};
        for (const PedStat& r : st) if (r.type != kSpeedNone && r.type != kSpeedPausy) { sum[r.type] += r.moving_mean(); ++cnt[r.type]; }
        if (cnt[kSpeedHurry] > 0 && cnt[kSpeedAsIs] > 0 && cnt[kSpeedSlow] > 0 &&
            sum[kSpeedHurry] / cnt[kSpeedHurry] > sum[kSpeedAsIs] / cnt[kSpeedAsIs] && sum[kSpeedAsIs] / cnt[kSpeedAsIs] > sum[kSpeedSlow] / cnt[kSpeedSlow])
          ++seeds_ok;
        for (int t = 1; t <= 4; ++t) { sum_c2[t] += sum[t]; cnt_c2[t] += cnt[t]; }
      }
    }
  }
  std::printf("  C1 hurry %d/%d (min %.2f) slow %d/%d (max %.2f)  C2 seeds %d/20 (crowd mean slow %.2f asis %.2f hurry %.2f)  C3 pausy %d/%d  all-stopped max %.1f s\n",
              ok_hurry, n_hurry, min_hurry, ok_slow, n_slow, max_slow, seeds_ok,
              sum_c2[kSpeedSlow] / std::max(1, cnt_c2[kSpeedSlow]), sum_c2[kSpeedAsIs] / std::max(1, cnt_c2[kSpeedAsIs]),
              sum_c2[kSpeedHurry] / std::max(1, cnt_c2[kSpeedHurry]), ok_pausy, n_pausy, worst_stopped);
  EXPECT_TRUE(worst_stopped < 60.0);   // D: 全員が 60 s 以上動かなくならない(窓は kStopWindow = 120 s。制御器なしの代理。制御ありの走行は到達で見る)
  EXPECT_TRUE(n_hurry > 0 && n_slow > 0 && n_pausy > 0);
  // C の閾値(plan の暫定値をそのまま確定。変えない)
  EXPECT_TRUE(n_hurry > 0 && ok_hurry >= 0.8 * n_hurry);   // C1 hurry: 80% 以上が ≥ 1.5 m/s
  EXPECT_TRUE(n_slow > 0 && ok_slow >= 0.8 * n_slow);      // C1 slow: 80% 以上が ≤ 0.8 m/s(速さの上限 0.77 で必ず通る。検査力なし)
  EXPECT_TRUE(seeds_ok >= 18);                             // C2: hurry > そのまま > slow の走行が 18/20 以上
  EXPECT_TRUE(n_pausy > 0 && ok_pausy >= 0.8 * n_pausy);   // C3: 80% 以上で停止時間 ≥ 窓の 10%
}

MINI_TEST_MAIN()
