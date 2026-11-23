#pragma once
// 速さの違う人(候補 8。記録のみ・既定 off)。Scenario を後処理する。makeScenario の rng は消費しない(配置・goal・人数は off と同じ)。
#include <cstdint>

#include "sim/scenarios.hpp"

namespace social_nav {
namespace sim {

// speed_type(PedInit)
constexpr int kSpeedNone = 0;    // 対象外
constexpr int kSpeedSlow = 1;    // v0 = U(0.5, 0.7)
constexpr int kSpeedHurry = 2;   // v0 = U(1.8, 2.0)
constexpr int kSpeedPausy = 3;   // 周期停止(v0 は元のまま)
constexpr int kSpeedAsIs = 4;    // 対象だが何も変えない

// 対象: corridor_oncoming / crossing / crowd の、frozen でない・kind == 0・group_id < 0 の人。ほかは何もしない。
// 呼び出し側が seed + 8647 を渡す。人 1 人につき 5 個(種別、v0、T_w、T_p、位相)を同じ順で引く
void applySpeedMix(Scenario& sc, uint32_t mix_seed);

}  // namespace sim
}  // namespace social_nav
