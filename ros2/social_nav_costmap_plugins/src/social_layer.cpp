#include "social_nav_costmap_plugins/social_layer.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "tf2/time.hpp"
#include "tf2_ros/buffer.h"

namespace social_nav_costmap_plugins {

void SocialLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("SocialLayer: failed to lock node");
  }

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("people_topic", rclcpp::ParameterValue(std::string("/social_nav/tracked_people")));
  declareParameter("max_cost", rclcpp::ParameterValue(200));
  declareParameter("sigma", rclcpp::ParameterValue(0.45));
  declareParameter("velocity_gain", rclcpp::ParameterValue(0.4));
  declareParameter("rear_ratio", rclcpp::ParameterValue(0.6));
  declareParameter("cutoff_cost", rclcpp::ParameterValue(0.05));
  declareParameter("max_speed_for_layer", rclcpp::ParameterValue(0.3));
  declareParameter("prediction_time", rclcpp::ParameterValue(0.0));
  declareParameter("stale_timeout", rclcpp::ParameterValue(0.5));
  declareParameter("min_confidence", rclcpp::ParameterValue(0.3));

  bool enabled = true;
  std::string topic;
  int max_cost = 200;
  double stale_timeout = 0.5;
  node->get_parameter(getFullName("enabled"), enabled);
  node->get_parameter(getFullName("people_topic"), topic);
  node->get_parameter(getFullName("max_cost"), max_cost);
  node->get_parameter(getFullName("sigma"), raster_.prox.sigma);
  node->get_parameter(getFullName("velocity_gain"), raster_.prox.velocity_gain);
  node->get_parameter(getFullName("rear_ratio"), raster_.prox.rear_ratio);
  node->get_parameter(getFullName("cutoff_cost"), raster_.prox.cutoff);
  node->get_parameter(getFullName("max_speed_for_layer"), raster_.max_speed);
  node->get_parameter(getFullName("prediction_time"), raster_.prediction_time);
  node->get_parameter(getFullName("stale_timeout"), stale_timeout);
  node->get_parameter(getFullName("min_confidence"), raster_.min_confidence);
  raster_.max_cost = static_cast<uint8_t>(std::clamp(max_cost, 0, 252));

  enabled_ = enabled;
  current_ = true;
  global_frame_ = layered_costmap_->getGlobalFrameID();
  // 鮮度・信頼度の絞り込みは購読ヘルパで行う(描画側の min_confidence は二重適用だが無害)
  people_sub_ = std::make_unique<social_nav_ros_common::PeopleSubscriber>(
    node, topic, stale_timeout, raster_.min_confidence);
}

social_nav::TrackedPeople SocialLayer::currentPeople()
{
  social_nav::TrackedPeople p = people_sub_->get();
  if (p.agents.empty() || p.frame_id.empty() || p.frame_id == global_frame_) {
    return p;
  }
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_->lookupTransform(global_frame_, p.frame_id, tf2::TimePointZero);
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(logger_, *clock_, 5000, "SocialLayer: TF %s -> %s failed: %s",
                         p.frame_id.c_str(), global_frame_.c_str(), e.what());
    p.agents.clear();
    return p;
  }
  const auto & q = tf.transform.rotation;
  const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  const double c = std::cos(yaw), s = std::sin(yaw);
  for (auto & a : p.agents) {
    const social_nav::Vec2 pos = a.pos, vel = a.vel;
    a.pos = {c * pos.x - s * pos.y + tf.transform.translation.x,
             s * pos.x + c * pos.y + tf.transform.translation.y};
    a.vel = {c * vel.x - s * vel.y, s * vel.x + c * vel.y};
    // 共分散 R C R^T
    const double xx = a.pos_cov.xx, xy = a.pos_cov.xy, yy = a.pos_cov.yy;
    a.pos_cov.xx = c * c * xx - 2 * c * s * xy + s * s * yy;
    a.pos_cov.xy = c * s * (xx - yy) + (c * c - s * s) * xy;
    a.pos_cov.yy = s * s * xx + 2 * c * s * xy + c * c * yy;
  }
  p.frame_id = global_frame_;
  return p;
}

void SocialLayer::updateBounds(double, double, double, double * min_x, double * min_y,
                               double * max_x, double * max_y)
{
  // 無効化直後も前回描いた範囲は一度だけ広げて消す
  people_ = enabled_ ? currentPeople() : social_nav::TrackedPeople{};
  const social_nav::Aabb now = social_nav::socialBounds(people_, raster_);

  const social_nav::Aabb * const boxes[2] = {&prev_aabb_, &now};
  for (const social_nav::Aabb * b : boxes) {
    if (b->valid) {
      *min_x = std::min(*min_x, b->min_x);
      *min_y = std::min(*min_y, b->min_y);
      *max_x = std::max(*max_x, b->max_x);
      *max_y = std::max(*max_y, b->max_y);
    }
  }
  prev_aabb_ = now;
}

void SocialLayer::updateCosts(nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j,
                              int max_i, int max_j)
{
  if (!enabled_ || people_.agents.empty()) {
    return;
  }
  const int sx = static_cast<int>(master_grid.getSizeInCellsX());
  const int sy = static_cast<int>(master_grid.getSizeInCellsY());
  social_nav::CellWindow win{std::max(min_i, 0), std::max(min_j, 0), std::min(max_i, sx) - 1,
                             std::min(max_j, sy) - 1};
  if (win.max_i < win.min_i || win.max_j < win.min_j) {
    return;
  }
  const social_nav::GridSpec grid{master_grid.getOriginX(), master_grid.getOriginY(),
                                  master_grid.getResolution(), static_cast<unsigned>(sx),
                                  static_cast<unsigned>(sy)};
  social_nav::rasterizeSocialCost(grid, win, people_, raster_, master_grid.getCharMap());
}

void SocialLayer::reset()
{
  // 次回 updateBounds で前回分を消せるよう prev_aabb_ は残し、保持中の人だけ捨てる
  people_ = {};
}

}  // namespace social_nav_costmap_plugins

PLUGINLIB_EXPORT_CLASS(social_nav_costmap_plugins::SocialLayer, nav2_costmap_2d::Layer)
