#pragma once
#include <cmath>
#include <algorithm>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "social_nav_core/sampling_controller.hpp"
#include "social_nav_core/types.hpp"

namespace social_nav_controller {

// accel_rollout の ROS 側の解決。真かつ accel_clear_time <= 0 なら偽にして disabled を真にする
// (待ち時間なしの加速軌道は、人の入力の欠落を「人なし」と区別できないため)
inline bool resolveAccelRollout(bool accel_rollout, double accel_clear_time, bool & disabled)
{
  disabled = accel_rollout && !(accel_clear_time > 0.0);
  return accel_rollout && !disabled;
}

// core のタイマは 1 回の呼び出しを dt 秒と数える。制御周期(1/frequency)が dt とずれていれば真
inline bool accelPeriodMismatch(double controller_frequency, double dt)
{
  return !(controller_frequency > 0.0) || std::abs(1.0 / controller_frequency - dt) > 1e-3;
}

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

// ---- door_yield の静的障害物コスト(地図の距離場から sim と同じ式で作る) ----
// sim(social_nav_core/sim/main.cpp の makeObstacleFn)と同じ定数。core は変えないので ROS 側に複製し、単体テストで値を固定する
constexpr double kDoorRobotRadius = 0.3;
constexpr double kDoorInflationMargin = 0.05;
constexpr double kDoorCostDecay = 0.25;

// 占有セルの中心までの最短距離 [m]。格子は map フレーム(地図の origin の yaw は 0 とみなす)
struct DistanceField
{
  int width{0}, height{0};
  double resolution{0.05};
  double origin_x{0.0}, origin_y{0.0};
  std::vector<float> dist;  // width * height、行優先。占有セルが 1 つも無ければ全て大きい値
};

namespace detail {
// Felzenszwalb の 1 次元の距離変換(二乗距離)。f: 入力(0 または inf)、d: 出力
inline void edt1d(const std::vector<double> & f, std::vector<double> & d, std::vector<int> & v,
                  std::vector<double> & z)
{
  const int n = static_cast<int>(f.size());
  int k = 0;
  v[0] = 0;
  z[0] = -1e30;
  z[1] = 1e30;
  for (int q = 1; q < n; ++q) {
    double sx;
    while (true) {
      const int p = v[k];
      sx = ((f[q] + static_cast<double>(q) * q) - (f[p] + static_cast<double>(p) * p)) / (2.0 * (q - p));
      if (sx <= z[k]) {
        --k;  // z[0] = -1e30 なので k は 0 未満にならない
      } else {
        break;
      }
    }
    ++k;
    v[k] = q;
    z[k] = sx;
    z[k + 1] = 1e30;
  }
  k = 0;
  for (int q = 0; q < n; ++q) {
    while (z[k + 1] < q) {
      ++k;
    }
    d[q] = static_cast<double>(q - v[k]) * (q - v[k]) + f[v[k]];
  }
}
}  // namespace detail

// 厳密なユークリッド距離変換(行と列の 2 パス)。占有 = data >= thresh(unknown = -1 は占有でない)
inline std::shared_ptr<const DistanceField> buildDistanceField(
  const nav_msgs::msg::OccupancyGrid & grid, int occupied_thresh)
{
  auto f = std::make_shared<DistanceField>();
  const int w = static_cast<int>(grid.info.width), h = static_cast<int>(grid.info.height);
  f->width = w;
  f->height = h;
  f->resolution = grid.info.resolution;
  f->origin_x = grid.info.origin.position.x;
  f->origin_y = grid.info.origin.position.y;
  f->dist.assign(static_cast<std::size_t>(std::max(w, 0)) * std::max(h, 0), 1e6f);
  if (w <= 0 || h <= 0 || !(grid.info.resolution > 0.0) ||
    grid.data.size() < static_cast<std::size_t>(w) * h)
  {
    f->width = f->height = 0;
    f->dist.clear();
    return f;
  }
  const double inf = 1e12;  // 二乗距離の上限(1e18 だと倍精度で q^2 が落ちる)
  std::vector<double> g(static_cast<std::size_t>(w) * h);
  for (std::size_t i = 0; i < g.size(); ++i) {
    g[i] = (grid.data[i] >= 0 && grid.data[i] >= occupied_thresh) ? 0.0 : inf;
  }
  const int m = std::max(w, h);
  std::vector<double> col(m), out(m), zz(m + 1);
  std::vector<int> vv(m);
  // 列方向
  for (int x = 0; x < w; ++x) {
    col.resize(h);
    out.resize(h);
    for (int y = 0; y < h; ++y) {col[y] = g[static_cast<std::size_t>(y) * w + x];}
    detail::edt1d(col, out, vv, zz);
    for (int y = 0; y < h; ++y) {g[static_cast<std::size_t>(y) * w + x] = out[y];}
  }
  // 行方向
  for (int y = 0; y < h; ++y) {
    col.resize(w);
    out.resize(w);
    for (int x = 0; x < w; ++x) {col[x] = g[static_cast<std::size_t>(y) * w + x];}
    detail::edt1d(col, out, vv, zz);
    for (int x = 0; x < w; ++x) {
      const double d2 = out[x];
      f->dist[static_cast<std::size_t>(y) * w + x] =
        d2 >= inf * 0.5 ? 1e6f : static_cast<float>(std::sqrt(d2) * grid.info.resolution);
    }
  }
  return f;
}

// sim と同じ式。d: 壁(占有セル)までの距離 [m]、r: ロボット半径
inline double staticCostFromDistance(double d, double r = kDoorRobotRadius)
{
  if (d < r + kDoorInflationMargin) {
    return 1.0;
  }
  return 0.9 * std::exp(-(d - r - kDoorInflationMargin) / kDoorCostDecay);
}

// 呼び出し側のフレーム(odom)の点 p を map へ p_map = R(yaw) p + (tx, ty) で変換して距離場を引く。
// 地図の外は 0(障害物なし)。field は呼び出し側が生存を保証しなくてよい(共有所有)
inline social_nav::ObstacleCostFn makeStaticCostFn(
  std::shared_ptr<const DistanceField> field, double tx, double ty, double yaw,
  double r = kDoorRobotRadius)
{
  const double c = std::cos(yaw), s = std::sin(yaw);
  return [field, tx, ty, c, s, r](double x, double y) -> double {
           if (!field || field->width <= 0) {
             return 0.0;
           }
           const double mx = c * x - s * y + tx, my = s * x + c * y + ty;
           const double fx = std::floor((mx - field->origin_x) / field->resolution);
           const double fy = std::floor((my - field->origin_y) / field->resolution);
           if (!(fx >= 0.0 && fy >= 0.0 && fx < field->width && fy < field->height)) {
             return 0.0;
           }
           const double d = field->dist[static_cast<std::size_t>(fy) * field->width + static_cast<std::size_t>(fx)];
           return staticCostFromDistance(d, r);
         };
}

// 地図未受信・TF 失敗のとき渡す。常に 0(狭所点なし = 発動しない)。
// 空の ObstacleCostFn を渡すと core が obstacles(人を含む)に戻るので、必ず非空にする
inline social_nav::ObstacleCostFn makeZeroCostFn()
{
  return [](double, double) -> double {return 0.0;};
}

}  // namespace social_nav_controller
