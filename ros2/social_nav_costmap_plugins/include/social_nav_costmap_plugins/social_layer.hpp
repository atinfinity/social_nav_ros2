#pragma once
#include <memory>
#include <string>

#include "nav2_costmap_2d/layer.hpp"
#include "social_nav_core/grid_rasterizer.hpp"
#include "social_nav_core/types.hpp"
#include "social_nav_ros_common/people_subscriber.hpp"

namespace social_nav_costmap_plugins {

// 人の周りの非致死コストを max マージで描く戦略層。ロボット形状・駆動方式に依存しない。
class SocialLayer : public nav2_costmap_2d::Layer
{
public:
  SocialLayer() = default;
  ~SocialLayer() override = default;

  void onInitialize() override;
  void updateBounds(double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y,
                    double * max_x, double * max_y) override;
  void updateCosts(nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i,
                   int max_j) override;
  void reset() override;
  bool isClearable() override { return false; }

private:
  // 最新 people を global frame に変換して返す(変換不能なら空)
  social_nav::TrackedPeople currentPeople();

  std::unique_ptr<social_nav_ros_common::PeopleSubscriber> people_sub_;
  social_nav::RasterParams raster_;
  std::string global_frame_;

  // updateBounds で確定した今回の人と、前回描いた AABB(前回分を消すため和をとる)
  social_nav::TrackedPeople people_;
  social_nav::Aabb prev_aabb_;
};

}  // namespace social_nav_costmap_plugins
