#pragma once
#include <cstddef>
#include <memory>
#include <vector>

#include "social_nav_core/footprint.hpp"
#include "social_nav_core/proxemics.hpp"
#include "social_nav_core/types.hpp"

namespace social_nav {

// 反応層: SocialCritic と SamplingController が共用する唯一の社会的コスト
// yaw は nullptr 可(その場合 clearance は向き 0 で評価。円形では結果が変わらない)
struct TrajectoryView { const float* x; const float* y; const float* yaw; std::size_t n; std::ptrdiff_t stride; };

struct SocialCostParams { ProxemicsParams prox; double collision_margin{0.1};
                          double collision_cost{1000.0}; double time_decay_tau{2.0};
                          int step_subsample{2}; double max_range{6.0}; double min_confidence{0.3}; };

class SocialTrajectoryCost {
 public:
  SocialTrajectoryCost(SocialCostParams p, std::shared_ptr<const Footprint> footprint);
  // ロボット位置から max_range 内の人だけ残し、各評価ステップの予測位置を前計算する
  void prepare(const TrackedPeople& people, Vec2 robot_pos, double t_offset, double dt, std::size_t n_steps);
  double score(const TrajectoryView& traj) const;   // 小さいほど良い。prepare 済みが前提。スレッド安全(const)
  // 診断: 軌跡が全ステップのどこかで人(予測)と接触(score の衝突項と同じ条件)するか。prepare 済みが前提
  bool hasPredictedContact(const TrajectoryView& traj) const;
  // 診断用(cand_probe)。score と同じ走査で近接(proxemic)項と衝突項を分けて返す。score == proxemic + collision
  void scoreParts(const TrajectoryView& traj, double& proxemic, double& collision) const;
  bool empty() const;                               // 対象の人がいなければ true(呼び出し側で早期 return)

 private:
  struct Person { double radius; double reach; std::vector<Agent> predicted; };  // predicted[k] = ステップ k の予測
  SocialCostParams p_;
  std::shared_ptr<const Footprint> footprint_;
  std::vector<Person> people_;
  std::vector<double> weights_;   // exp(-t_k / tau)
};

}  // namespace social_nav
