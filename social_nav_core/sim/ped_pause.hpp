#pragma once
// 周期停止(候補 8「速さの違う人」の pausy)。時刻だけで決まる: 歩く T_w → 減速(0.8 m/s^2)→ 止まる T_p → 元の v0 で再開、を繰り返す。
// 歩行者モデルの本体は変えず、main のループが v0 だけを差し替える(goal には触れない。crowd の loop の人の goal 差し替えと衝突しない)。乱数は引かない。
// main.cpp と test_sim_scenarios.cpp が同じ実装を使う(複製しない)。
#include <algorithm>

#include "sim/social_force.hpp"

namespace social_nav {
namespace sim {

struct PedPauseSpec {
  double period{0.0};   // 歩く時間 T_w [s]。0 なら何もしない(既定)
  double pause{0.0};    // 止まる時間 T_p [s]
  double phase{0.0};    // 最初に止まり始める時刻(歩く時間の位相)[s]。実際は max(phase, kPauseNoStopBefore)
  double decel{0.8};    // v0 を下げる率 [m/s^2]
};

struct PedPauseState {
  int phase{0};          // 0 歩く / 1 減速 / 2 停止
  double t_next{-1.0};   // phase 0 で、次に減速を始める時刻(-1 = 未設定)
  double t_phase{0.0}, v0_orig{0.0};
  int n_stops{0};        // 減速を始めた回数
  double t_first_stop{-1.0};
};

constexpr double kPauseNoStopBefore = 3.0;   // この時刻より前には止まり始めない [s]
constexpr double kPauseV0Floor = 0.05;

// t = 現在時刻 [s]、dt = 1 ステップ。人 1 人ごとに毎物理ステップ、歩行者の step の前に呼ぶ
inline void stepPedPause(const PedPauseSpec& s, PedPauseState& st, Pedestrian& p, double t, double dt) {
  if (!(s.period > 0.0)) return;
  if (st.t_next < 0.0) st.t_next = std::max(s.phase, kPauseNoStopBefore);
  if (st.phase == 0) {
    if (t < st.t_next) return;
    st.phase = 1;
    st.t_phase = t;
    st.v0_orig = p.v0;
    ++st.n_stops;
    if (st.t_first_stop < 0.0) st.t_first_stop = t;
  }
  if (st.phase == 1) {
    p.v0 = std::max(kPauseV0Floor, p.v0 - s.decel * dt);
    if (t - st.t_phase >= st.v0_orig / s.decel) { st.phase = 2; st.t_phase = t; }
    return;
  }
  if (t - st.t_phase >= s.pause) {   // phase 2 → 再開
    p.v0 = st.v0_orig;
    st.phase = 0;
    st.t_next = t + s.period;
  }
}

}  // namespace sim
}  // namespace social_nav
