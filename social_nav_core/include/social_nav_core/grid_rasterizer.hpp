#pragma once
#include <cstdint>

#include "social_nav_core/proxemics.hpp"
#include "social_nav_core/types.hpp"

namespace social_nav {

// 戦略層: SocialLayer の中身(ロボット形状に依存しない)
struct GridSpec { double origin_x, origin_y, resolution; unsigned size_x, size_y; };   // index = y*size_x + x
struct CellWindow { int min_i, min_j, max_i, max_j; };                                // 両端を含む
struct Aabb { double min_x, min_y, max_x, max_y; bool valid{false}; };
struct RasterParams { ProxemicsParams prox; uint8_t max_cost{200}; double max_speed{0.3};
                      double prediction_time{0.0}; double min_confidence{0.3}; };

// 描画対象の人(confidence >= min_confidence かつ |v| <= max_speed)の描画範囲を包む AABB。
// prediction_time > 0 なら現在位置と予測位置の両方を含む。対象がいなければ valid=false
Aabb socialBounds(const TrackedPeople&, const RasterParams&);
// max マージ。値 253 以上と 255 のセルは変更しない
void rasterizeSocialCost(const GridSpec&, const CellWindow&, const TrackedPeople&, const RasterParams&,
                         uint8_t* cells);

}  // namespace social_nav
