#include "social_nav_core/types.hpp"

namespace social_nav {

Agent predictConstantVelocity(const Agent& a, double dt) {
  constexpr double kSigmaV = 0.3;
  Agent out = a;
  out.pos.x = a.pos.x + a.vel.x * dt;
  out.pos.y = a.pos.y + a.vel.y * dt;
  const double add = kSigmaV * kSigmaV * dt * dt;
  out.pos_cov.xx = a.pos_cov.xx + add;
  out.pos_cov.yy = a.pos_cov.yy + add;
  return out;
}

}  // namespace social_nav
