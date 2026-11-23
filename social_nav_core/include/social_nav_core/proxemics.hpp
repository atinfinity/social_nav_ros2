#pragma once
#include "social_nav_core/types.hpp"

namespace social_nav {

struct ProxemicsParams { double sigma{0.45}; double velocity_gain{0.4}; double rear_ratio{0.6};
                         double static_speed{0.2}; double cutoff{0.05}; };

// 人の進行方向を x' 軸とする非対称ガウス。|v| < static_speed なら等方。値域 [0,1]
double proxemicCost(const Agent& a, Vec2 p, const ProxemicsParams& prm);
// cost < cutoff となる最大距離
double proxemicRadius(const Agent& a, const ProxemicsParams& prm);

}  // namespace social_nav
