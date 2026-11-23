#pragma once
#include <cstdint>
#include <optional>
#include <vector>

#include "social_nav_core/crowd_analyzer.hpp"
#include "social_nav_core/types.hpp"

namespace social_nav {

// 戦術層の本体。BT ノードとシミュレータが同じものを使う
enum class Behavior { PROCEED, FOLLOW, YIELD, WAIT };
const char* toString(Behavior);

struct TacticalParams {
  CorridorParams corridor;
  double v_max{0.8};
  double density_radius{3.0}, density_low{0.1}, density_high{0.5}, min_ratio{0.4};   // PROCEED・FOLLOW・cooldown の 3 つの下限を兼ねる
  double blocker_max_speed{0.3}, max_yield_time{5.0}, yield_cooldown{10.0};          // YIELD
  double follow_min_speed{0.5}, follow_heading_tol{0.8}, follow_gap{1.5};            // FOLLOW(heading は rad)
  double density_wait{0.8}, density_resume{0.6}, max_wait_time{15.0};                // WAIT(ヒステリシス)
  double min_dwell{1.0};                                                             // 状態の最短継続時間
};

struct TacticalInput { double now; Pose2D robot; const std::vector<Pose2D>* path; const TrackedPeople* people; };
struct TacticalDecision { Behavior behavior{Behavior::PROCEED}; double speed_ratio{1.0};
                          std::optional<uint32_t> target_id; };

class TacticalSelector {
 public:
  explicit TacticalSelector(TacticalParams);
  TacticalDecision step(const TacticalInput& in);
  void reset();                       // 新しいゴール受信時

 private:
  TacticalParams p_;
  Behavior state_{Behavior::PROCEED};
  double since_;                      // 現在の状態に入った時刻
  double yield_block_until_;          // この時刻までは YIELD に入らない
  double wait_block_until_;           // この時刻までは WAIT に入らない(PROCEED は min_ratio)
  std::optional<uint32_t> target_;
};

}  // namespace social_nav
