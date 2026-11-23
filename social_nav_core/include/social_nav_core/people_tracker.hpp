#pragma once
#include <array>
#include <string>
#include <vector>

#include "social_nav_core/cluster_detector.hpp"
#include "social_nav_core/types.hpp"

namespace social_nav {

struct TrackerParams { double gate_mahalanobis{3.0}; double process_noise_acc{1.0}; int confirm_hits{5};
                       double delete_after{1.0}; double max_speed{3.0};
                       double init_vel_sigma{1.0};   // 新規 track の初期速度の標準偏差 [m/s](平均は 0)
                     };

class PeopleTracker {
 public:
  explicit PeopleTracker(TrackerParams p);
  // 予測 → ゲート付きの最適割り当て(逐次最短増加路法) → 残りを広いゲートで再対応(128 超は貪欲のみ) → KF 更新 → 生成/削除
  void update(const std::vector<Detection>& dets, double stamp);
  TrackedPeople confirmed(const std::string& frame_id) const;

 private:
  struct Track {
    uint32_t id{0};
    std::array<double, 4> x{};    // [x, y, vx, vy]
    std::array<double, 16> P{};   // 4x4 行優先
    int hits{0};
    double last_update{0};
    double radius{0.3};
  };
  TrackerParams p_;
  std::vector<Track> tracks_;
  uint32_t next_id_{1};
  double stamp_{0};
  bool has_stamp_{false};
};

}  // namespace social_nav
