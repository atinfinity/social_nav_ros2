#include "social_nav_core/proxemics.hpp"

#include <algorithm>
#include <cmath>

namespace social_nav {

double proxemicCost(const Agent& a, Vec2 p, const ProxemicsParams& prm) {
  const double rx = p.x - a.pos.x;
  const double ry = p.y - a.pos.y;
  const double speed = std::hypot(a.vel.x, a.vel.y);
  const double sy = prm.sigma;
  double dx = rx, dy = ry, sx = prm.sigma;
  if (speed >= prm.static_speed) {
    const double c = a.vel.x / speed, s = a.vel.y / speed;
    dx = c * rx + s * ry;
    dy = -s * rx + c * ry;
    sx = dx > 0.0 ? prm.sigma + prm.velocity_gain * speed : prm.sigma * prm.rear_ratio;
  }
  const double e = dx * dx / (2.0 * sx * sx) + dy * dy / (2.0 * sy * sy);
  return std::min(1.0, std::max(0.0, std::exp(-e)));
}

double proxemicRadius(const Agent& a, const ProxemicsParams& prm) {
  const double speed = std::hypot(a.vel.x, a.vel.y);
  double smax = prm.sigma;
  if (speed >= prm.static_speed) {
    smax = std::max({prm.sigma, prm.sigma + prm.velocity_gain * speed, prm.sigma * prm.rear_ratio});
  }
  return smax * std::sqrt(2.0 * std::log(1.0 / prm.cutoff));
}

}  // namespace social_nav
