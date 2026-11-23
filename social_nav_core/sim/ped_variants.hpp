#pragma once
// sfm2 用の歩行者属性(家族・歩きスマホ)の付与。Scenario を後処理する。makeScenario の rng は消費しない。
#include <cstdint>

#include "sim/scenarios.hpp"

namespace social_nav {
namespace sim {

// crowd だけに付ける(家族 3 組 = 2・3・2 人、歩きスマホ 4 人)。ほかのシナリオは何もしない。
// 選ぶ人は seed(呼び出し側が seed+4243 を渡す)から作る専用 engine で決める。
void applyPedVariants(Scenario& sc, uint32_t ped_seed);

}  // namespace sim
}  // namespace social_nav
