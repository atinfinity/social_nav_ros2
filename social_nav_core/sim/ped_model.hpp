#pragma once
// 歩行者モデル sfm2: Moussaïd 型の先読み Social Force(相手との相対速度を見る相互作用項)。ROS 非依存。
// 旧 SocialForceWorld(social_force.hpp)とは別クラス。Pedestrian / RobotBody / Segment は共有する。
#include <cstdint>
#include <random>
#include <vector>

#include "sim/social_force.hpp"

namespace social_nav {
namespace sim {

// group_id がこの値以上の群れ(group_oncoming)だけ、視線項の向きだけ Moussaid 2010 の形(f = -beta alpha V。速度に沿う減速)にする。係数、視野角、重心の定義は論文と異なる(docs/ped_model.md)。
// それ未満(crowd の 0..2)は従来の横加速度の式のまま(crowd の出力を変えないため。一致の確認はシード 1–20、aarch64 の既定ビルド)。2 式の併存は暫定で、
// 統一(全群を論文形に)は判定 21–120 を回すときに決める。
constexpr int kPaperGazeGroupIdMin = 100;

struct PedParams {
  // 駆動項
  double tau{0.5};          // s。文献値(Moussaïd 2009/2010)
  double max_speed{2.0};    // m/s。数値安全用の上限(文献値ではない)
  double speed_cap{1.1};    // 個人の速さの上限 = speed_cap * v0(調整 2 で追加。先行者が押されて v0 を超えると、流れの平均速度が下がらないため)
  // 相互作用項(Moussaïd 2010 PLoS ONE の力の形式)。文献値は記憶に基づく。未確認(docs/ped_model.md で確認する)
  double A{10.0};           // m/s^2。文献値 4.5。調整 1 で 7、調整 2 で 10(docs/ped_model.md の調整履歴)
  double gamma{0.4};        // B = gamma * |D|。文献値 0.35。調整 1 で 0.45、調整 2 で 0.4
  double n{2.0};            // 横方向(回避)の角度減衰
  double n_prime{3.0};      // 減速方向の角度減衰
  double lambda{2.0};       // D = lambda (v_a - v_b) + e_ab
  double fov_deg{200.0};    // 視野の全角 [deg]。目標方向から外れた相手(視野外)の相互作用に rear_weight を掛ける(Helbing & Molnár 1995 の形。値は記憶ベースで未確認)
  double rear_weight{0.65}; // 視野外の重み c。調整 3 で追加(一方向流で前後が対称だと密度で減速しないため)
  double range{6.0};        // m。これより遠い相手は無視(打ち切り。文献値ではない)
  // 壁・接触(文献に定量値が薄い。初期値であり調整対象)
  double A_wall{3.0};       // m/s^2
  double B_wall{0.3};       // m
  double k_contact{500.0};  // 1/s^2。体の重なりに対するばね(legacy と同じ値)
  // OU 雑音(加速度 [m/s^2] に加える。文献値ではない)
  double noise_sigma{0.3};  // 定常標準偏差
  double noise_tau{0.5};    // s。相関時間
  // 群れ(Moussaïd 2010 の視線・凝集・群内斥力)。形は文献どおりだが係数値は未確認(記憶ベース)。初期値であり調整対象
  double group_vis_gain{1.0};     // 1/s^2。視線(重心が視野外のとき進行方向を重心へ向ける横加速度)
  double group_fov_deg{100.0};    // 視野の全角 [deg]
  double group_attract{3.0};      // 1/s^2。凝集(重心までの距離が閾値超のとき重心へ)
  double group_dist_free{0.7};    // m。凝集が効き始める重心距離 = group_dist_free * (N-1)/2 + group_dist_base
  double group_dist_base{0.4};    // m
  double group_repulse{3.0};      // m/s^2。群内斥力の強さ
  double group_repulse_dist{0.6}; // m。これより近い仲間から離れる向きに押す(体の接触より外側)
  double group_vis_gain_paper{1.0}; // 1/s。group_id >= kPaperGazeGroupIdMin の群れの視線(論文の形: 速度に沿う減速 -beta*(|alpha|-視野半角)*V)の係数
  double group_inter_scale{0.3};  // 群内の相互作用項(先読み回避)に掛ける係数。群内では弱める
  // 歩きスマホ(kind==2)。文献の定量値は薄く、初期値で動画を見て決める(未確認)
  double phone_v0_scale{0.8};     // v0 に掛ける
  double phone_attention{0.4};    // 注意係数に掛ける
  double phone_noise_scale{2.0};  // OU 雑音の振幅に掛ける
};

class PedModel {
 public:
  // noise_seed: 歩行者専用 engine の seed(呼び出し側が seed+4242 を渡す)。他の engine とは共有しない。
  PedModel(PedParams prm, std::vector<Segment> walls, uint32_t noise_seed);
  // robot が nullptr ならロボットを無視。ロボットは robot->vel つきの 1 エージェント扱い。
  // 乱数は呼び出しごとに peds.size() 人 x 2 個の標準正規(= engine 生出力 4 個/人)を、配列順に固定で引く。
  void step(std::vector<Pedestrian>& peds, const RobotBody* robot, double dt);
  // a が b から受ける相互作用の加速度(注意係数は掛けない)。テストと計測用。
  Vec2 interactionForce(Vec2 pos_a, Vec2 vel_a, Vec2 pos_b, Vec2 vel_b) const;
  const std::vector<Segment>& walls() const { return walls_; }

 private:
  void normalPair(double& z0, double& z1);   // mt19937 の生出力 2 個 -> Box-Muller 2 値
  PedParams p_;
  std::vector<Segment> walls_;
  std::mt19937 eng_;
  std::vector<Vec2> noise_;   // 人ごとの OU 状態(配列順)
};

}  // namespace sim
}  // namespace social_nav
