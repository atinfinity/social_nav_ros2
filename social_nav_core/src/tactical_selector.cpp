#include "social_nav_core/tactical_selector.hpp"

#include <algorithm>
#include <cmath>

namespace social_nav {

namespace {
constexpr double kNever = -1e18;
}

const char* toString(Behavior b) {
  switch (b) {
    case Behavior::PROCEED: return "PROCEED";
    case Behavior::FOLLOW: return "FOLLOW";
    case Behavior::YIELD: return "YIELD";
    case Behavior::WAIT: return "WAIT";
  }
  return "UNKNOWN";
}

TacticalSelector::TacticalSelector(TacticalParams p) : p_(p) { reset(); }

void TacticalSelector::reset() {
  state_ = Behavior::PROCEED;
  since_ = kNever;
  yield_block_until_ = kNever;
  wait_block_until_ = kNever;
  target_.reset();
}

TacticalDecision TacticalSelector::step(const TacticalInput& in) {
  const double now = in.now;
  TacticalDecision d;

  // people が null/空でも状態機械は「人なし」入力として通常どおり評価する
  // (min_dwell・上限時間・cooldown・since_ を飛ばさない)
  static const TrackedPeople kNoPeople;
  const TrackedPeople& people = in.people ? *in.people : kNoPeople;
  static const std::vector<Pose2D> kNoPath;
  const std::vector<Pose2D>& path = in.path ? *in.path : kNoPath;

  const double density = crowdDensity(people, {in.robot.x, in.robot.y}, p_.density_radius);
  const double dens_ratio = speedLimitRatio(density, p_.density_low, p_.density_high, p_.min_ratio);
  const auto on_path = peopleOnPath(path, in.robot, people, p_.corridor);
  const PersonOnPath* near = on_path.empty() ? nullptr : &on_path.front();

  // 上限時間(min_dwell より優先)
  bool forced = false;
  if (state_ == Behavior::WAIT && now - since_ >= p_.max_wait_time) {
    wait_block_until_ = now + p_.yield_cooldown;
    forced = true;
  }
  if (state_ == Behavior::YIELD && now - since_ >= p_.max_yield_time) {
    yield_block_until_ = now + p_.yield_cooldown;
    forced = true;
  }

  // 優先順位: WAIT > YIELD > FOLLOW > PROCEED
  const bool wait_want = (state_ == Behavior::WAIT && !forced) ? density >= p_.density_resume
                                                               : (density > p_.density_wait && now >= wait_block_until_);
  const bool yield_want = near && near->speed < p_.blocker_max_speed && now >= yield_block_until_;
  bool follow_want = false;
  if (near && near->speed > 1e-9 && near->speed_along >= p_.follow_min_speed) {
    const double c = std::min(1.0, std::max(-1.0, near->speed_along / near->speed));
    follow_want = std::acos(c) < p_.follow_heading_tol;
  }
  Behavior desired = wait_want ? Behavior::WAIT
                     : yield_want ? Behavior::YIELD
                     : follow_want ? Behavior::FOLLOW : Behavior::PROCEED;

  if (desired != state_ && (forced || now - since_ >= p_.min_dwell)) {
    state_ = desired;
    since_ = now;
  }

  d.behavior = state_;
  switch (state_) {
    case Behavior::WAIT:
      d.speed_ratio = 0.0;
      target_.reset();
      break;
    case Behavior::YIELD:
      d.speed_ratio = 0.0;
      if (near && near->speed < p_.blocker_max_speed) target_ = near->id;
      d.target_id = target_;
      break;
    case Behavior::FOLLOW: {
      if (near) {
        double r = std::min(std::max(near->speed_along / p_.v_max, p_.min_ratio), dens_ratio);
        if (near->dist_along < p_.follow_gap) r *= std::max(0.0, near->dist_along) / p_.follow_gap;
        d.speed_ratio = r;
        target_ = near->id;
      } else {
        d.speed_ratio = dens_ratio;
      }
      d.target_id = target_;
      break;
    }
    case Behavior::PROCEED:
      // 空でも cooldown 中は min_ratio(空は密度 0 なので cooldown 外では dens_ratio=1.0)
      d.speed_ratio = now < wait_block_until_ ? p_.min_ratio : dens_ratio;
      target_.reset();
      break;
  }
  return d;
}

}  // namespace social_nav
