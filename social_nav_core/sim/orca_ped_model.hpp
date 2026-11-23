#pragma once
// 歩行者モデル orca: ORCA(Optimal Reciprocal Collision Avoidance。van den Berg, Guy, Lin & Manocha 2011)。ROS 非依存。
// 速度障害物の半平面(ORCA 線)を作り、希望速度に最も近い速度を線形計画で選ぶ。決定的(乱数を使わない)。
// 線形計画と壁の線の手順は RVO2 Library(Apache-2.0)の src/Agent.cc に基づく。由来と変更点は orca_ped_model.cpp の冒頭と NOTICE。sfm2(PedModel)、ttc は変えない。
#include <vector>

#include "sim/social_force.hpp"

namespace social_nav {
namespace sim {

struct OrcaParams {
  // RVO2 同梱の Circle 例(examples/Circle.cc の setAgentDefaults)の値。固定。接触数を見て変えない
  double neighbor_dist{15.0};      // m
  int max_neighbors{10};
  double time_horizon{10.0};       // s(歩行者・ロボット)
  double time_horizon_obst{5.0};   // s(壁)。RVO2 の壁のある例(Blocks、Roadmap)の値。10 s から変更(測定前に固定。結果で調整しない)
  double speed_cap{1.1};           // 最大速度 = speed_cap * v0(sfm2 と同じ)
  double ped_responsibility{0.5};  // 歩行者同士は半分ずつ避ける
  double robot_responsibility{1.0};  // ロボットは ORCA に従わない。歩行者が全量を避ける
  double arrive_dist{0.3};         // m。これより近い目標に着いたら停止(sfm2 と同じ)
};

class OrcaPedModel {
 public:
  OrcaPedModel(OrcaParams prm, std::vector<Segment> walls);
  // robot が nullptr ならロボットを無視。全員の新しい速度を、更新前の位置・速度から同時に求めてから進める。
  void step(std::vector<Pedestrian>& peds, const RobotBody* robot, double dt) const;

 private:
  OrcaParams p_;
  std::vector<Segment> walls_;
};

}  // namespace sim
}  // namespace social_nav
