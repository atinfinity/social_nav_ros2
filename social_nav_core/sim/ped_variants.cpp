#include "sim/ped_variants.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <random>
#include <vector>

namespace social_nav {
namespace sim {

namespace {
// crowd の壁(hallWalls(-4, 26, -4, 4))の内側。人の半径ぶん内側(makeScenario の y 範囲 ±3.5 と同じ)
constexpr double kXMin = -3.5, kXMax = 25.5, kYMax = 3.5;
}  // namespace

void applyPedVariants(Scenario& sc, uint32_t ped_seed) {
  if (sc.name != "crowd") return;
  std::mt19937 eng(ped_seed);
  std::vector<std::size_t> idx;
  for (std::size_t i = 0; i < sc.peds.size(); ++i)
    if (!sc.peds[i].frozen) idx.push_back(i);
  // 自前の Fisher-Yates(std::shuffle は実装依存のため。eng の生出力だけ使う)
  for (std::size_t i = idx.size(); i > 1; --i) std::swap(idx[i - 1], idx[eng() % i]);

  const int family_sizes[3] = {2, 3, 2};
  std::size_t next = 0;
  for (int g = 0; g < 3; ++g) {
    if (next + family_sizes[g] > idx.size()) return;
    PedInit& anchor = sc.peds[idx[next]];
    const double hx = anchor.goal.x - anchor.pos.x, hy = anchor.goal.y - anchor.pos.y;
    const double hd = std::hypot(hx, hy);
    const Vec2 perp = hd > 1e-9 ? Vec2{-hy / hd, hx / hd} : Vec2{0.0, 1.0};
    // 並べる側は +perp 側を先に試し、だめなら反対側。どちらも makeScenario と同じ制約
    // (始点から 1.5 m 超、家族以外から 0.8 m 超、壁の内側)を満たせなければ家族にしない
    std::vector<Vec2> placed;
    bool ok_side = false;
    for (int side = 0; side < 2 && !ok_side; ++side) {
      placed.clear();
      ok_side = true;
      for (int k = 1; k < family_sizes[g] && ok_side; ++k) {
        const double off = 0.7 * ((k + 1) / 2) * ((k % 2 == 1) == (side == 0) ? 1.0 : -1.0);   // 横並び
        const Vec2 q{anchor.pos.x + perp.x * off, anchor.pos.y + perp.y * off};
        ok_side = q.x >= kXMin && q.x <= kXMax && std::fabs(q.y) <= kYMax &&
                  std::hypot(q.x - sc.start.x, q.y - sc.start.y) > 1.5;
        for (std::size_t j = 0; j < sc.peds.size() && ok_side; ++j) {
          bool in_family = (j == idx[next]);
          for (int m = 1; m < family_sizes[g]; ++m) in_family = in_family || (j == idx[next + m]);
          if (!in_family) ok_side = std::hypot(q.x - sc.peds[j].pos.x, q.y - sc.peds[j].pos.y) > 0.8;
        }
        placed.push_back(q);
      }
    }
    if (ok_side) {
      anchor.kind = 1;
      anchor.group_id = g;
      for (int k = 1; k < family_sizes[g]; ++k) {
        PedInit& m = sc.peds[idx[next + k]];
        m.pos = placed[k - 1];
        m.goal = anchor.goal;
        m.v0 = anchor.v0;
        m.kind = 1;
        m.group_id = g;
      }
    }
    next += static_cast<std::size_t>(family_sizes[g]);
  }
  for (int k = 0; k < 4 && next < idx.size(); ++k, ++next) sc.peds[idx[next]].kind = 2;
}

}  // namespace sim
}  // namespace social_nav
