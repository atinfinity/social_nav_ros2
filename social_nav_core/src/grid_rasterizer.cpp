#include "social_nav_core/grid_rasterizer.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace social_nav {
namespace {

bool drawable(const Agent& a, const RasterParams& prm) {
  if (a.confidence < prm.min_confidence) return false;
  return std::hypot(a.vel.x, a.vel.y) <= prm.max_speed;
}

// 現在位置(と prediction_time > 0 なら予測位置)の人を列挙
void collect(const TrackedPeople& people, const RasterParams& prm, std::vector<Agent>& out) {
  for (const Agent& a : people.agents) {
    if (!drawable(a, prm)) continue;
    out.push_back(a);
    if (prm.prediction_time > 0.0) out.push_back(predictConstantVelocity(a, prm.prediction_time));
  }
}

}  // namespace

Aabb socialBounds(const TrackedPeople& people, const RasterParams& prm) {
  std::vector<Agent> as;
  collect(people, prm, as);
  Aabb box{0, 0, 0, 0, false};
  for (const Agent& a : as) {
    const double r = proxemicRadius(a, prm.prox);
    if (!box.valid) {
      box = {a.pos.x - r, a.pos.y - r, a.pos.x + r, a.pos.y + r, true};
    } else {
      box.min_x = std::min(box.min_x, a.pos.x - r);
      box.min_y = std::min(box.min_y, a.pos.y - r);
      box.max_x = std::max(box.max_x, a.pos.x + r);
      box.max_y = std::max(box.max_y, a.pos.y + r);
    }
  }
  return box;
}

void rasterizeSocialCost(const GridSpec& g, const CellWindow& win, const TrackedPeople& people,
                         const RasterParams& prm, uint8_t* cells) {
  if (cells == nullptr || g.size_x == 0 || g.size_y == 0 || g.resolution <= 0.0) return;
  const int i0 = std::max(win.min_i, 0), j0 = std::max(win.min_j, 0);
  const int i1 = std::min(win.max_i, static_cast<int>(g.size_x) - 1);
  const int j1 = std::min(win.max_j, static_cast<int>(g.size_y) - 1);
  if (i0 > i1 || j0 > j1) return;

  std::vector<Agent> as;
  collect(people, prm, as);
  const double peak = std::min<double>(prm.max_cost, 252.0);   // 253 以上は致死/unknown 領域
  for (const Agent& a : as) {
    const double r = proxemicRadius(a, prm.prox);
    const int ai0 = std::max(i0, static_cast<int>(std::floor((a.pos.x - r - g.origin_x) / g.resolution)));
    const int ai1 = std::min(i1, static_cast<int>(std::floor((a.pos.x + r - g.origin_x) / g.resolution)));
    const int aj0 = std::max(j0, static_cast<int>(std::floor((a.pos.y - r - g.origin_y) / g.resolution)));
    const int aj1 = std::min(j1, static_cast<int>(std::floor((a.pos.y + r - g.origin_y) / g.resolution)));
    for (int j = aj0; j <= aj1; ++j) {
      for (int i = ai0; i <= ai1; ++i) {
        uint8_t& c = cells[static_cast<size_t>(j) * g.size_x + static_cast<size_t>(i)];
        if (c >= 253) continue;   // 致死・内接・unknown は変更しない
        const Vec2 p{g.origin_x + (i + 0.5) * g.resolution, g.origin_y + (j + 0.5) * g.resolution};
        const double cost = proxemicCost(a, p, prm.prox);
        if (cost < prm.prox.cutoff) continue;
        const uint8_t v = static_cast<uint8_t>(std::lround(std::min(cost, 1.0) * peak));
        if (v > c) c = v;
      }
    }
  }
}

}  // namespace social_nav
