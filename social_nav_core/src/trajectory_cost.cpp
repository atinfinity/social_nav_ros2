#include "social_nav_core/trajectory_cost.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace social_nav {

SocialTrajectoryCost::SocialTrajectoryCost(SocialCostParams p, std::shared_ptr<const Footprint> footprint)
    : p_(std::move(p)), footprint_(std::move(footprint)) {
  if (!footprint_) throw std::invalid_argument("SocialTrajectoryCost: footprint must not be null");
}

void SocialTrajectoryCost::prepare(const TrackedPeople& people, Vec2 robot_pos, double t_offset, double dt,
                                   std::size_t n_steps) {
  people_.clear();
  weights_.assign(n_steps, 0.0);
  for (std::size_t k = 0; k < n_steps; ++k) {
    const double t = t_offset + static_cast<double>(k) * dt;
    weights_[k] = std::exp(-t / p_.time_decay_tau);
  }
  for (const Agent& a : people.agents) {
    if (a.confidence < p_.min_confidence) continue;
    if (std::hypot(a.pos.x - robot_pos.x, a.pos.y - robot_pos.y) > p_.max_range) continue;
    Person person;
    person.radius = a.radius;
    // 到達半径: 衝突判定または快適性が届く最大距離。等速予測で速度は不変なので k によらない
    person.reach = std::max(footprint_->circumscribedRadius() + a.radius + p_.collision_margin, proxemicRadius(a, p_.prox));
    person.predicted.reserve(n_steps);
    for (std::size_t k = 0; k < n_steps; ++k)
      person.predicted.push_back(predictConstantVelocity(a, t_offset + static_cast<double>(k) * dt));
    people_.push_back(std::move(person));
  }
}

bool SocialTrajectoryCost::empty() const { return people_.empty(); }

bool SocialTrajectoryCost::hasPredictedContact(const TrajectoryView& traj) const {
  const std::size_t n = std::min(traj.n, weights_.size());
  const double circ = footprint_->circumscribedRadius();
  for (std::size_t k = 0; k < n; ++k) {
    const std::ptrdiff_t off = static_cast<std::ptrdiff_t>(k) * traj.stride;
    const Pose2D pose{traj.x[off], traj.y[off], traj.yaw ? static_cast<double>(traj.yaw[off]) : 0.0};
    for (const Person& person : people_) {
      const Agent& pa = person.predicted[k];
      const double lim = person.radius + p_.collision_margin;
      const double d = std::hypot(pa.pos.x - pose.x, pa.pos.y - pose.y);
      if (d <= circ + lim && footprint_->clearance(pose, pa.pos) < lim) return true;
    }
  }
  return false;
}

double SocialTrajectoryCost::score(const TrajectoryView& traj) const {
  if (people_.empty() || traj.n == 0) return 0.0;
  const std::size_t n = std::min(traj.n, weights_.size());
  const std::size_t step = static_cast<std::size_t>(std::max(1, p_.step_subsample));
  const double circ = footprint_->circumscribedRadius();
  double total = 0.0;
  for (std::size_t k = 0; k < n; k += step) {
    const std::ptrdiff_t off = static_cast<std::ptrdiff_t>(k) * traj.stride;
    const Pose2D pose{traj.x[off], traj.y[off], traj.yaw ? static_cast<double>(traj.yaw[off]) : 0.0};
    const Vec2 pos{pose.x, pose.y};
    double sum = 0.0;
    for (const Person& person : people_) {
      const Agent& pa = person.predicted[k];
      const double lim = person.radius + p_.collision_margin;
      const double d = std::hypot(pa.pos.x - pose.x, pa.pos.y - pose.y);
      if (d > person.reach) continue;  // 事前判定(衝突も快適性も届かないなら省略)
      sum += proxemicCost(pa, pos, p_.prox);
      if (d <= circ + lim && footprint_->clearance(pose, pa.pos) < lim) sum += p_.collision_cost;
    }
    total += weights_[k] * sum;
  }
  return total;
}

void SocialTrajectoryCost::scoreParts(const TrajectoryView& traj, double& proxemic, double& collision) const {
  proxemic = 0.0;
  collision = 0.0;
  if (people_.empty() || traj.n == 0) return;
  const std::size_t n = std::min(traj.n, weights_.size());
  const std::size_t step = static_cast<std::size_t>(std::max(1, p_.step_subsample));
  const double circ = footprint_->circumscribedRadius();
  for (std::size_t k = 0; k < n; k += step) {
    const std::ptrdiff_t off = static_cast<std::ptrdiff_t>(k) * traj.stride;
    const Pose2D pose{traj.x[off], traj.y[off], traj.yaw ? static_cast<double>(traj.yaw[off]) : 0.0};
    const Vec2 pos{pose.x, pose.y};
    for (const Person& person : people_) {
      const Agent& pa = person.predicted[k];
      const double lim = person.radius + p_.collision_margin;
      const double d = std::hypot(pa.pos.x - pose.x, pa.pos.y - pose.y);
      if (d > person.reach) continue;
      proxemic += weights_[k] * proxemicCost(pa, pos, p_.prox);
      if (d <= circ + lim && footprint_->clearance(pose, pa.pos) < lim) collision += weights_[k] * p_.collision_cost;
    }
  }
}

}  // namespace social_nav
