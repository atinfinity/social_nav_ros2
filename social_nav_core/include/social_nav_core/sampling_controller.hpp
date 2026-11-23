#pragma once
#include <functional>
#include <memory>
#include <vector>

#include "social_nav_core/footprint.hpp"
#include "social_nav_core/motion_model.hpp"
#include "social_nav_core/trajectory_cost.hpp"
#include "social_nav_core/types.hpp"

namespace social_nav {

// 反応層: シミュレータと SocialController(3.4)が共用
using ObstacleCostFn = std::function<double(double x, double y)>;   // [0,1)、1 以上は衝突

struct ControllerParams { VelocityLimits limits; SampleSpec samples; double horizon{3.0}, dt{0.1};
                          double w_path{1.0}, w_goal{2.0}, w_obstacle{5.0}, w_social{5.0}, w_speed{0.2};
                          SocialCostParams social;
                          bool stop_aware{false};   // R2: 通過の候補に空きがあるとき、接触する減速の候補を棄却する(既定 off = 現行)
                          bool tie_min_turn{false};   // 譲るときの 1 周の案 T: コストが同点(1e-9 以内)なら |wz| が小さい候補を選ぶ(既定 off = 現行の厳密比較)
                          bool tie_min_turn_stopped{false};  // 譲るときの 1 周の案 T2: tie_min_turn と同じ同点の解消を、speed_limit_ratio == 0(YIELD / WAIT)の周期だけに掛ける(既定 off)
                          bool tie_path_heading{false};  // 譲るときの 1 周の案 H: コストが同点(1e-9 以内)なら、終点の向きが現在位置に最も近い経路区間の接線に近い候補を選ぶ(経路 2 点未満、tie_min_turn* が on のときは無効。既定 off)
                          double w_turn{0.0};      // 譲るときの 1 周の案 A: 候補の |wz| への重み(既定 0 = 現行)
                          double w_heading{0.0};   // 譲るときの 1 周の案 C: 終点の向きと終点から目標への方位のずれ [rad] への重み(既定 0 = 現行)
                          bool accel_rollout{false};   // accel_rollout: 人が 6 m 以内にいない周期だけ、各 wz で vx 最大の候補を加速継続の軌道でも評価し、低い方のコストを採る(既定 off = 現行。人の情報の欠落も「人なし」と扱うので、実機では有効にしない)
};

// all_colliding のとき cmd は零指令、best_cost は 0
// 診断(setDiagnostics(true) のときだけ埋める。cmd の選択には使わない)。予測接触 = 3 s 先まで人の予測と外形が接触
struct ControlDiag {
  bool valid{false};
  int n_cand{0};            // 障害物で棄却されなかった候補の本数
  int n_free{0};            // うち予測接触のない本数
  int zero_free{-1};        // その場停止(零指令)の軌跡に予測接触がないか(1/0)。人がいなければ -1
  int chosen_contact{-1};   // 選ばれた候補に予測接触があったか(1/0)。選べなければ -1
};

struct ControlResult { Twist2D cmd; bool all_colliding{false}; double best_cost{0}; ControlDiag diag; };

class SamplingController {
 public:
  SamplingController(ControllerParams, std::shared_ptr<const MotionModel>, std::shared_ptr<const Footprint>);
  ControlResult compute(const Pose2D& pose, const Twist2D& vel, const std::vector<Pose2D>& path,
                        const TrackedPeople& people, const ObstacleCostFn& obstacles,
                        double speed_limit_ratio = 1.0);
  void setDiagnostics(bool on) { diag_ = on; }

 private:
  ControllerParams p_;
  std::shared_ptr<const MotionModel> model_;
  std::shared_ptr<const Footprint> footprint_;
  SocialTrajectoryCost social_;
  bool diag_{false};
};

}  // namespace social_nav
