#pragma once
#include <atomic>
#include <memory>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "social_nav_controller/controller_compat.hpp"
#include "social_nav_controller/controller_utils.hpp"
#include "social_nav_core/sampling_controller.hpp"
#include "social_nav_ros_common/people_subscriber.hpp"

namespace social_nav_controller {

class SocialController : public nav2_core::Controller
{
public:
  SocialController() = default;
  ~SocialController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setPlan(const nav_msgs::msg::Path & path) override;
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;
  void setSpeedLimit(compat::SpeedLimitArg speed_limit, compat::PercentageArg percentage) override;
  void reset() override;

private:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("SocialController")};
  rclcpp::Clock::SharedPtr clock_;
  std::string name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  std::unique_ptr<social_nav::SamplingController> controller_;
  std::unique_ptr<social_nav_ros_common::PeopleSubscriber> people_;
  nav_msgs::msg::Path plan_;
  double v_max_{0.8};
  double path_lookahead_{3.0};
  double stuck_timeout_{2.0};
  std::atomic<double> speed_limit_ratio_{1.0};  // setSpeedLimit(別スレッド)と compute から参照
  bool colliding_{false};
  rclcpp::Time colliding_since_{0, 0, RCL_ROS_TIME};

  // door_yield(既定 false)。false のときは購読も TF 参照も作らず、compute の static_obstacles は空のまま
  bool door_yield_{false};
  int door_occupied_thresh_{65};
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr door_map_sub_;
  std::mutex door_map_mutex_;
  std::shared_ptr<const DistanceField> door_field_;  // door_map_mutex_ で守る
  std::string door_map_frame_;                       // 同上
};

}  // namespace social_nav_controller
