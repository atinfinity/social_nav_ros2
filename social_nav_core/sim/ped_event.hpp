#pragma once
// 急に止まる・振り返る人(記録のみ)。歩行者モデルの本体は変えず、main のループが v0 と goal だけを差し替える状態機械。
// 走行 → 減速(v0 を一定率で下げる)→ 停止(goal = pos)→ 再開(goal を差し替え、v0 を戻す)。乱数は引かない。
// main.cpp と test_sim_scenarios.cpp が同じ実装を使う(複製しない)。
#include <algorithm>
#include <cmath>

#include "sim/social_force.hpp"

namespace social_nav {
namespace sim {

struct PedEventSpec {
  int kind{0};            // 0 なし(既定。goal / v0 に触れない) / 1 stop / 2 turn / 3 reverse
  double trigger_d{2.0};  // 前方距離 ped.x - robot.x がこれ以下で発火 [m]
  double t_max{12.0};     // 未発火ならこの時刻で発火 [s]
  double decel{2.0};      // v0 を下げる率 [m/s^2]
  double pause{0.5};      // 停止してから再開するまで [s]
  double turn_y{1.2};     // turn の再開先 (停止位置の x, turn_y)(符号つき)
  double reverse_x{-4.0}; // reverse の再開先 (reverse_x, 停止位置の y)
};

struct PedEventState {
  int phase{0};           // 0 未発火 / 1 減速 / 2 停止 / 3 再開済み
  double t_fire{-1.0}, d_fire{-1.0}, t_phase{0.0}, v0_orig{0.0};
  Vec2 orig_goal{};
  Vec2 pos_fire{};   // 発火位置(turn / reverse の再開先はこれを使う)
};

constexpr double kEventV0Floor = 0.05;

// robot = ロボットの位置、t = 現在時刻 [s]、dt = 1 ステップ。人 1 人ごとに毎物理ステップ、歩行者の step の前に呼ぶ
inline void stepPedEvent(const PedEventSpec& s, PedEventState& st, Pedestrian& p, Vec2 robot, double t, double dt) {
  if (s.kind == 0 || st.phase == 3) return;
  if (st.phase == 0) {
    if (p.pos.x - robot.x > s.trigger_d && t < s.t_max) return;
    st.phase = 1;
    st.t_fire = t;
    st.d_fire = std::hypot(p.pos.x - robot.x, p.pos.y - robot.y);
    st.t_phase = t;
    st.v0_orig = p.v0;
    st.orig_goal = p.goal;
    st.pos_fire = p.pos;
  }
  if (st.phase == 1) {
    p.v0 = std::max(kEventV0Floor, p.v0 - s.decel * dt);
    if (t - st.t_phase >= st.v0_orig / s.decel) {
      p.goal = p.pos;
      st.phase = 2;
      st.t_phase = t;
    }
    return;
  }
  if (t - st.t_phase >= s.pause) {   // phase 2
    p.v0 = st.v0_orig;
    if (s.kind == 1) p.goal = st.orig_goal;
    else if (s.kind == 2) p.goal = {st.pos_fire.x, s.turn_y};
    else p.goal = {s.reverse_x, st.pos_fire.y};
    st.phase = 3;
  }
}

}  // namespace sim
}  // namespace social_nav
