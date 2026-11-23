#include "social_nav_perception/people_tracker_node.hpp"

#include <chrono>
#include <vector>

#include "social_nav_perception/perception_utils.hpp"
#include "social_nav_ros_common/conversions.hpp"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

namespace social_nav_perception {

namespace {
social_nav::ClusterParams declareCluster(rclcpp::Node & n)
{
  social_nav::ClusterParams p;
  p.z_min = n.declare_parameter("cluster.z_min", p.z_min);
  p.z_max = n.declare_parameter("cluster.z_max", p.z_max);
  p.cell = n.declare_parameter("cluster.cell", p.cell);
  p.min_width = n.declare_parameter("cluster.min_width", p.min_width);
  p.max_width = n.declare_parameter("cluster.max_width", p.max_width);
  p.max_range = n.declare_parameter("cluster.max_range", p.max_range);
  p.min_points = static_cast<int>(n.declare_parameter("cluster.min_points", static_cast<int64_t>(p.min_points)));
  return p;
}

social_nav::TrackerParams declareTracker(rclcpp::Node & n)
{
  social_nav::TrackerParams p;
  p.gate_mahalanobis = n.declare_parameter("tracker.gate_mahalanobis", p.gate_mahalanobis);
  p.process_noise_acc = n.declare_parameter("tracker.process_noise_acc", p.process_noise_acc);
  p.confirm_hits = static_cast<int>(n.declare_parameter("tracker.confirm_hits", static_cast<int64_t>(p.confirm_hits)));
  p.delete_after = n.declare_parameter("tracker.delete_after", p.delete_after);
  p.max_speed = n.declare_parameter("tracker.max_speed", p.max_speed);
  p.init_vel_sigma = n.declare_parameter("tracker.init_vel_sigma", p.init_vel_sigma);
  return p;
}
}  // namespace

PeopleTrackerNode::PeopleTrackerNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("people_tracker_node", options),
  base_frame_(declare_parameter("base_frame", std::string("base_link"))),
  odom_frame_(declare_parameter("odom_frame", std::string("odom"))),
  map_frame_(declare_parameter("map_frame", std::string("map"))),
  map_exclusion_radius_(declare_parameter("map_exclusion_radius", 0.2)),
  map_occupied_threshold_(static_cast<int>(declare_parameter("map_occupied_threshold", static_cast<int64_t>(65)))),
  tf_timeout_(declare_parameter("tf_timeout", 0.1)),
  detector_(declareCluster(*this)),
  tracker_(declareTracker(*this))
{
  const auto cloud_topic = declare_parameter("cloud_topic", std::string("points"));
  const auto map_topic = declare_parameter("map_topic", std::string("map"));
  const auto output_topic = declare_parameter("output_topic", std::string("/social_nav/tracked_people"));
  const double publish_rate = declare_parameter("publish_rate", 10.0);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    cloud_topic, rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::PointCloud2::SharedPtr m) {onCloud(m);});
  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    map_topic, rclcpp::QoS(1).transient_local().reliable(),
    [this](nav_msgs::msg::OccupancyGrid::SharedPtr m) {onMap(m);});
  pub_ = create_publisher<social_nav_msgs::msg::TrackedPeople>(output_topic, 10);
  timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / publish_rate), [this]() {publish();});
}

void PeopleTrackerNode::onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  map_ = msg;
}

void PeopleTrackerNode::onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  const auto stamp = tf2_ros::fromMsg(msg->header.stamp);
  const auto timeout = tf2::durationFromSec(tf_timeout_);
  sensor_msgs::msg::PointCloud2 cloud_base;
  geometry_msgs::msg::TransformStamped base_in_odom_tf;
  try {
    cloud_base = tf_buffer_->transform(*msg, base_frame_, timeout);
    base_in_odom_tf = tf_buffer_->lookupTransform(odom_frame_, base_frame_, stamp, timeout);
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF unavailable, skipping cloud: %s", e.what());
    return;
  }
  social_nav::Pose2D base_in_odom;
  base_in_odom.x = base_in_odom_tf.transform.translation.x;
  base_in_odom.y = base_in_odom_tf.transform.translation.y;
  base_in_odom.theta = yawOf(base_in_odom_tf.transform.rotation);

  auto dets = detector_.detect(cloudToPoints(cloud_base), base_in_odom);

  nav_msgs::msg::OccupancyGrid::SharedPtr map;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    map = map_;
  }
  if (map && !dets.empty()) {
    // 検出は odom 座標。地図近傍の判定のため odom -> map へ変換する。変換できなければ除外しない。
    try {
      const auto odom_in_map = tf_buffer_->lookupTransform(map_frame_, odom_frame_, stamp, timeout);
      const double yaw = yawOf(odom_in_map.transform.rotation);
      const double c = std::cos(yaw), s = std::sin(yaw);
      const auto & t = odom_in_map.transform.translation;
      std::vector<social_nav::Detection> kept;
      for (const auto & d : dets) {
        const double mx = c * d.pos.x - s * d.pos.y + t.x;
        const double my = s * d.pos.x + c * d.pos.y + t.y;
        if (!nearOccupied(*map, mx, my, map_exclusion_radius_, map_occupied_threshold_)) {
          kept.push_back(d);
        }
      }
      dets.swap(kept);
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "map TF unavailable, no map filtering: %s", e.what());
    }
  }

  const double t = social_nav_ros_common::toSeconds(msg->header.stamp);
  std::lock_guard<std::mutex> lock(mutex_);
  tracker_.update(dets, t);
  last_stamp_ = t;
  has_stamp_ = true;
}

void PeopleTrackerNode::publish()
{
  social_nav::TrackedPeople people;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    people = tracker_.confirmed(odom_frame_);
    // 点群が来ていない間は now を刻む(受信側は stamp が古ければ無視する)
    people.stamp = has_stamp_ ? last_stamp_ : now().seconds();
  }
  pub_->publish(social_nav_ros_common::toMsg(people));
}

}  // namespace social_nav_perception
