#include "social_nav_core/footprint.hpp"

#include <cmath>
#include <stdexcept>

namespace social_nav {

CircleFootprint::CircleFootprint(double radius) : radius_(radius) {
  if (!(radius > 0.0)) throw std::invalid_argument("CircleFootprint: radius must be positive");
}

double CircleFootprint::inscribedRadius() const { return radius_; }
double CircleFootprint::circumscribedRadius() const { return radius_; }

double CircleFootprint::clearance(const Pose2D& robot, Vec2 p) const {
  return std::hypot(p.x - robot.x, p.y - robot.y) - radius_;
}

void CircleFootprint::costQueryPoints(const Pose2D& robot, std::vector<Vec2>& out) const {
  out.push_back(Vec2{robot.x, robot.y});
}

std::unique_ptr<Footprint> makeFootprint(const FootprintSpec& spec) {
  if (spec.type == "circle") return std::make_unique<CircleFootprint>(spec.radius);
  throw std::invalid_argument("makeFootprint: unsupported footprint type: " + spec.type);
}

}  // namespace social_nav
