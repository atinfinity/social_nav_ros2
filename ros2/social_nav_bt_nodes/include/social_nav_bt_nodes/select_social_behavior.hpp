#pragma once
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/msg/speed_limit.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "social_nav_bt_nodes/bt_compat.hpp"
#include "social_nav_core/tactical_selector.hpp"
#include "social_nav_ros_common/people_subscriber.hpp"
#include "tf2_ros/buffer.h"

namespace social_nav_bt
{

// ロボット姿勢(TF)・最新 people・path をコアの TacticalSelector に渡し、
// behavior を出力ポートへ、speed_ratio を speed_limit トピックへ出す。常に SUCCESS。
// パラメータは blackboard の "node" の social_behavior.* から読む(XML には書かない)。
class SelectSocialBehavior : public BT::SyncActionNode
{
public:
  SelectSocialBehavior(const std::string & name, const NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  void lazyInit();
  void publishSpeed(double ratio);
  void resetOnGoalChange();
  social_nav::TacticalDecision proceed() const;

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string global_frame_{"map"};
  std::string robot_frame_{"base_link"};
  std::unique_ptr<social_nav::TacticalSelector> selector_;
  // bt_navigator の client_node は誰も spin しないため、Nav2 標準ノードと同様に
  // 自動追加しない callback group を専用 executor で tick ごとに spin_some する。
  rclcpp::CallbackGroup::SharedPtr cb_group_;
  rclcpp::executors::SingleThreadedExecutor cb_executor_;
  std::unique_ptr<social_nav_ros_common::PeopleSubscriber> people_;
  rclcpp::Publisher<nav2_msgs::msg::SpeedLimit>::SharedPtr speed_pub_;
  BT::Blackboard::Ptr bb_;
  geometry_msgs::msg::PoseStamped last_goal_;
  bool has_goal_{false};
  double last_percentage_{100.0};
  bool initialized_{false};
};

}  // namespace social_nav_bt
