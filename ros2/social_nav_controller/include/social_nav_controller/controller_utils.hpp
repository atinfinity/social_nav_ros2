#pragma once
#include <cmath>
#include <cstddef>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "social_nav_core/sampling_controller.hpp"
#include "social_nav_core/types.hpp"

namespace social_nav_controller {

// ロボット最近傍点より手前を刈り、そこから累積弧長 lookahead までを返す(最近傍点を含む)
inline std::vector<social_nav::Pose2D> prunePath(const std::vector<social_nav::Pose2D> & path,
                                                 const social_nav::Pose2D & robot, double lookahead)
{
  std::vector<social_nav::Pose2D> out;
  if (path.empty()) {
    return out;
  }
  std::size_t best = 0;
  double best_d2 = 1e300;
  for (std::size_t i = 0; i < path.size(); ++i) {
    const double dx = path[i].x - robot.x, dy = path[i].y - robot.y;
    const double d2 = dx * dx + dy * dy;
    if (d2 < best_d2) {
      best_d2 = d2;
      best = i;
    }
  }
  double acc = 0.0;
  for (std::size_t i = best; i < path.size(); ++i) {
    if (i > best) {
      acc += std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
      if (acc > lookahead) {
        break;
      }
    }
    out.push_back(path[i]);
  }
  return out;
}

// 範囲外と NO_INFORMATION は 1.0(衝突扱い)、それ以外は cost/253
inline social_nav::ObstacleCostFn makeObstacleCostFn(const nav2_costmap_2d::Costmap2D & costmap)
{
  return [&costmap](double x, double y) -> double {
           unsigned int mx = 0, my = 0;
           if (!costmap.worldToMap(x, y, mx, my)) {
             return 1.0;
           }
           const unsigned char c = costmap.getCost(mx, my);
           if (c == nav2_costmap_2d::NO_INFORMATION) {
             return 1.0;
           }
           return static_cast<double>(c) / 253.0;
         };
}

// 2D 剛体変換(平行移動 + yaw)を人に適用する。位置は回転+平行移動、速度は回転のみ
inline social_nav::TrackedPeople transformPeople(const social_nav::TrackedPeople & in, double tx,
                                                 double ty, double yaw,
                                                 const std::string & new_frame)
{
  const double c = std::cos(yaw), s = std::sin(yaw);
  social_nav::TrackedPeople out = in;
  out.frame_id = new_frame;
  for (auto & a : out.agents) {
    a.pos = {c * a.pos.x - s * a.pos.y + tx, s * a.pos.x + c * a.pos.y + ty};
    a.vel = {c * a.vel.x - s * a.vel.y, s * a.vel.x + c * a.vel.y};
    const auto cv = a.pos_cov;
    a.pos_cov.xx = c * c * cv.xx - 2 * c * s * cv.xy + s * s * cv.yy;
    a.pos_cov.xy = c * s * (cv.xx - cv.yy) + (c * c - s * s) * cv.xy;
    a.pos_cov.yy = s * s * cv.xx + 2 * c * s * cv.xy + c * c * cv.yy;
  }
  return out;
}

}  // namespace social_nav_controller
