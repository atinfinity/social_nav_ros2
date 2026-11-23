#include "sim/ped_speed_mix.hpp"

#include <random>

namespace social_nav {
namespace sim {

namespace {
// 一様 [0,1) は mt19937 の 32 bit 出力から(std の distribution は実装依存のため使わない)
double unit(std::mt19937& eng) { return (static_cast<double>(eng()) + 0.5) / 4294967296.0; }

constexpr double kPSlow = 0.30, kPHurry = 0.30, kPPausy = 0.20;   // 残り 0.20 はそのまま
}  // namespace

void applySpeedMix(Scenario& sc, uint32_t mix_seed) {
  if (sc.name != "corridor_oncoming" && sc.name != "crossing" && sc.name != "crowd") return;
  std::mt19937 eng(mix_seed);
  for (PedInit& p : sc.peds) {
    if (p.frozen || p.kind != 0 || p.group_id >= 0) continue;
    const double u_type = unit(eng), u_v0 = unit(eng), u_w = unit(eng), u_p = unit(eng), u_ph = unit(eng);
    if (u_type < kPSlow) {
      p.speed_type = kSpeedSlow;
      p.v0 = 0.5 + 0.2 * u_v0;
    } else if (u_type < kPSlow + kPHurry) {
      p.speed_type = kSpeedHurry;
      p.v0 = 1.8 + 0.2 * u_v0;
    } else if (u_type < kPSlow + kPHurry + kPPausy) {
      p.speed_type = kSpeedPausy;
      p.pause.period = 4.0 + 4.0 * u_w;
      p.pause.pause = 1.5 + 1.5 * u_p;
      p.pause.phase = p.pause.period * u_ph;
    } else {
      p.speed_type = kSpeedAsIs;
    }
  }
}

}  // namespace sim
}  // namespace social_nav
