#pragma once
#include <memory>
#include <mutex>
#include <string>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "social_nav_core/cluster_detector.hpp"
#include "social_nav_core/people_tracker.hpp"
#include "social_nav_msgs/msg/tracked_people.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace social_nav_perception {

// PointCloud2 -> base_link -> LidarClusterDetector -> 静的地図近傍の除外 -> PeopleTracker -> odom で配信
class PeopleTrackerNode : public rclcpp::Node
{
public:
  explicit PeopleTrackerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void publish();

  std::string base_frame_, odom_frame_, map_frame_;
  double map_exclusion_radius_;
  int map_occupied_threshold_;
  double tf_timeout_;

  social_nav::LidarClusterDetector detector_;
  social_nav::PeopleTracker tracker_;
  double last_stamp_{0.0};
  bool has_stamp_{false};
  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
  std::mutex mutex_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<social_nav_msgs::msg::TrackedPeople>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace social_nav_perception
