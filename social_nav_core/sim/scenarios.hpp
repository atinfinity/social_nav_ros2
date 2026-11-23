#pragma once
// シナリオ定義(6 種)。乱数はシードから作った engine だけを使う。
#include <random>
#include <string>
#include <vector>

#include "sim/ped_event.hpp"
#include "sim/ped_pause.hpp"
#include "sim/social_force.hpp"
#include "social_nav_core/types.hpp"

namespace social_nav {
namespace sim {

struct PedInit {
  Vec2 pos, goal;
  double v0{1.3};
  bool frozen{false};
  bool loop{false};
  int kind{0};          // 0 通常 / 1 家族 / 2 歩きスマホ(ped_variants が付ける。makeScenario は付けない)
  int group_id{-1};
  bool has_via{false};  // true なら、まず via へ向かい、via の 0.5 m 以内か via を越えたら goal へ切り替える(main.cpp のループが行う。歩行者モデルは goal だけ見る)
  Vec2 via{};
  PedEventSpec event{};  // 急に止まる・振り返る人(候補 5)。kind 0(既定)なら何もしない。必ず末尾(位置指定の初期化子を壊さない)
  double release_t{0.0};  // > 0 なら、その時刻まで位置を変えず速度 0(frozen 扱い)で待ち、t >= release_t で目標へ歩き出す(候補 6。0 なら何もしない。必ず末尾)
  PedPauseSpec pause{};   // 周期停止(候補 8。period 0 なら何もしない。ped_speed_mix が付ける。必ず末尾)
  int speed_type{0};      // 速さの種別(候補 8。0 = 対象外。ped_speed_mix が付ける。意味は ped_speed_mix.hpp)
};

struct Scenario {
  std::string name;
  std::vector<Segment> walls;
  Pose2D start;
  Vec2 goal;
  std::vector<PedInit> peds;
  double timeout{120.0};
  double opening_w{-1.0};   // blind_corner の開口部の幅 [m](--blind-csv 用。他は -1)
};

const std::vector<std::string>& scenarioNames();       // 既定の 6 本(--scenario all)。変えない
const std::vector<std::string>& rearScenarioNames();   // 後方からの人流 3 本(--scenario rear で明示したときだけ。記録のみ)
const std::vector<std::string>& indoorScenarioNames(); // 屋内 3 本(door_oncoming, side_door, group_oncoming。--scenario indoor で明示したときだけ。記録のみ)
const std::vector<std::string>& blindScenarioNames();  // 壁の裏から出てくる人 1 本(blind_corner。--scenario blind で明示したときだけ。記録のみ)
const std::vector<std::string>& suddenScenarioNames(); // 急に止まる・振り返る人 3 本(sudden_stop, sudden_turn, sudden_reverse。--scenario sudden で明示したときだけ。記録のみ)
const std::vector<std::string>& elevatorScenarioNames(); // エレベーター前・出口の流れ 1 本(elevator_flow。--scenario elevator で明示したときだけ。記録のみ)
Scenario makeScenario(const std::string& name, std::mt19937& rng);   // 未知の名前は std::invalid_argument

}  // namespace sim
}  // namespace social_nav
