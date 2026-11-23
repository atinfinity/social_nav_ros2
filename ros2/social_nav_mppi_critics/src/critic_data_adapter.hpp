#pragma once
// nav2_mppi_controller の CriticData に触る唯一の場所(Jazzy: xtensor 行優先 / Kilted 以降: Eigen 列優先)。
// 版が変わったらこのファイルの関数だけを直す。
#include <cstddef>

#include "nav2_mppi_controller/critic_data.hpp"
#include "social_nav_core/trajectory_cost.hpp"

namespace social_nav_mppi_critics
{

// バッチ i 行目の軌道を (x, y, yaw) の view にする。stride は要素間の float 数
inline social_nav::TrajectoryView makeView(const mppi::CriticData & data, std::size_t i)
{
  const auto & t = data.trajectories;
  const std::size_t n = t.x.shape(1);
  const std::ptrdiff_t stride =
    n > 1 ? (&t.x(i, 1) - &t.x(i, 0)) : 1;   // 長さ 1 の次元の xtensor stride は 0 になり得るため
  return social_nav::TrajectoryView{&t.x(i, 0), &t.y(i, 0), &t.yaws(i, 0), n, stride};
}

inline void addCost(mppi::CriticData & data, std::size_t i, double c)
{
  data.costs(i) += static_cast<float>(c);
}

// score() が必要とするバッチ全体の情報(ロボット位置・ステップ数・dt・バッチ数)
struct BatchInfo { social_nav::Vec2 robot_pos; std::size_t batch; std::size_t steps; double dt; };

inline BatchInfo batchInfo(const mppi::CriticData & data)
{
  const auto & p = data.state.pose.pose.position;
  return BatchInfo{{p.x, p.y}, data.trajectories.x.shape(0), data.trajectories.x.shape(1),
    static_cast<double>(data.model_dt)};
}

}  // namespace social_nav_mppi_critics
