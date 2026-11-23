#pragma once
// Nav2 版差を閉じ込める互換ヘッダ。Jazzy の nav2_core::Controller に合わせてある(確認済み):
//   configure(const LifecycleNode::WeakPtr&, std::string, shared_ptr<tf2_ros::Buffer>, shared_ptr<Costmap2DROS>)
//   setSpeedLimit(const double&, const bool&)
//   computeVelocityCommands(const PoseStamped&, const Twist&, nav2_core::GoalChecker*)
// 他の版でシグネチャが変わったらここだけを直す。
#include <algorithm>

#include "nav2_core/controller.hpp"

namespace social_nav_controller::compat {

using SpeedLimitArg = const double &;
using PercentageArg = const bool &;

// Nav2 の NO_SPEED_LIMIT は 0.0。上限なし = 1.0。結果は [0,1]
inline double speedLimitRatio(double speed_limit, bool percentage, double v_max)
{
  if (speed_limit <= 0.0) {
    return 1.0;
  }
  const double ratio = percentage ? speed_limit / 100.0 : (v_max > 0.0 ? speed_limit / v_max : 1.0);
  return std::clamp(ratio, 0.0, 1.0);
}

}  // namespace social_nav_controller::compat
