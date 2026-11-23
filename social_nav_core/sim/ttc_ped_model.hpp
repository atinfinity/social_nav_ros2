#pragma once
// 歩行者モデル ttc: TTC べき乗則(Karamouzas, Skinner & Guy 2014, PRL 113, 238701)。ROS 非依存。
// 相互作用エネルギー E(tau) = (k / tau^2) exp(-tau / tau0)、力 = -grad E。tau は外形(円の半径の和)で測る衝突までの時間。
// sfm2(PedModel)は変えない。壁・接触ばね・OU 雑音・速さ上限・到達後の停止は PedParams の値をそのまま使う。
#include <cstdint>
#include <random>
#include <vector>

#include "sim/ped_model.hpp"

namespace social_nav {
namespace sim {

struct TtcParams {
  double k{1.5};      // m^2。論文の値(固定。接触数を見て変えない)
  double tau0{3.0};   // s。論文の値(固定)
  double f_max{10.0}; // m/s^2。1 組あたりの相互作用の加速度の上限(打ち切り。文献値ではない。tau の発散・かすり接触の勾配の発散を防ぐ)
  PedParams base;     // 駆動の緩和時間 tau(0.5 s。論文に値を確認できないため sfm2 の値)、壁、接触ばね、雑音、速さ上限
};

class TtcPedModel {
 public:
  TtcPedModel(TtcParams prm, std::vector<Segment> walls, uint32_t noise_seed);
  void step(std::vector<Pedestrian>& peds, const RobotBody* robot, double dt);
  // a が b から受ける相互作用の加速度。離れていく・かすらない・重なっている(外形が既に交差)ときは 0。テスト用。
  Vec2 interactionForce(Vec2 pos_a, Vec2 vel_a, double r_a, Vec2 pos_b, Vec2 vel_b, double r_b) const;

 private:
  void normalPair(double& z0, double& z1);
  TtcParams p_;
  std::vector<Segment> walls_;
  std::mt19937 eng_;
  std::vector<Vec2> noise_;
};

}  // namespace sim
}  // namespace social_nav
