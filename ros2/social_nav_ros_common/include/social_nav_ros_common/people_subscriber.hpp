#pragma once
#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "social_nav_core/types.hpp"
#include "social_nav_msgs/msg/tracked_people.hpp"
#include "social_nav_ros_common/conversions.hpp"

namespace social_nav_ros_common {

// 鮮度切れ(now - stamp > stale_timeout)なら空を返す。min_confidence 未満の人は除く。
inline social_nav::TrackedPeople freshPeople(const social_nav::TrackedPeople& latest, double now_sec,
                                             double stale_timeout, double min_confidence)
{
  social_nav::TrackedPeople out;
  out.stamp = latest.stamp;
  out.frame_id = latest.frame_id;
  if (now_sec - latest.stamp > stale_timeout) {
    return out;
  }
  for (const auto& a : latest.agents) {
    if (a.confidence >= min_confidence) {
      out.agents.push_back(a);
    }
  }
  return out;
}

// 最新の TrackedPeople を保持し、鮮度チェック付きで返す購読ヘルパ。
// NodePtr は rclcpp::Node / rclcpp_lifecycle::LifecycleNode の shared_ptr。
class PeopleSubscriber
{
public:
  template<class NodePtr>
  PeopleSubscriber(const NodePtr & node, const std::string & topic, double stale_timeout,
                   double min_confidence,
                   rclcpp::CallbackGroup::SharedPtr group = nullptr)
  : clock_(node->get_clock()), stale_timeout_(stale_timeout), min_confidence_(min_confidence)
  {
    rclcpp::SubscriptionOptions options;
    options.callback_group = group;  // nullptr ならノードの既定グループ
    sub_ = node->template create_subscription<social_nav_msgs::msg::TrackedPeople>(
      topic, rclcpp::QoS(10),
      [this](social_nav_msgs::msg::TrackedPeople::ConstSharedPtr msg) { update(*msg); }, options);
  }

  void update(const social_nav_msgs::msg::TrackedPeople & msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_ = toCore(msg);
    received_ = true;
  }

  // 未受信・鮮度切れなら空
  social_nav::TrackedPeople get() const { return get(clock_->now().seconds()); }

  social_nav::TrackedPeople get(double now_sec) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!received_) {
      return {};
    }
    return freshPeople(latest_, now_sec, stale_timeout_, min_confidence_);
  }

private:
  rclcpp::Clock::SharedPtr clock_;
  double stale_timeout_;
  double min_confidence_;
  mutable std::mutex mutex_;
  social_nav::TrackedPeople latest_;
  bool received_{false};
  rclcpp::Subscription<social_nav_msgs::msg::TrackedPeople>::SharedPtr sub_;
};

}  // namespace social_nav_ros_common
