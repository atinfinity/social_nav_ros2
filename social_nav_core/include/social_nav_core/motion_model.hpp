#pragma once
#include <memory>
#include <string>
#include <vector>

#include "social_nav_core/types.hpp"

namespace social_nav {

struct VelocityLimits { double v_min{0.0}, v_max{0.8}, vy_max{0.0}, w_max{1.5};
                        double acc_v{1.0}, acc_vy{0.0}, acc_w{3.0}; double min_turning_radius{0.0}; };
struct SampleSpec { int v_samples{9}, vy_samples{1}, w_samples{21}; };

class MotionModel {
 public:
  virtual ~MotionModel() = default;
  virtual const char* name() const = 0;
  virtual Pose2D integrate(const Pose2D& pose, const Twist2D& cmd, double dt) const = 0;
  // 現在速度から dt 後に到達可能で、limits とモデル固有の拘束を満たす指令候補を out に追加する
  virtual void sampleCommands(const Twist2D& current, const VelocityLimits& limits, const SampleSpec& spec,
                              double dt, std::vector<Twist2D>& out) const = 0;
  virtual bool canRotateInPlace() const = 0;
};

// vy=0。厳密な円弧積分(theta は折り返さない)。min_turning_radius と vy は無視する
class DiffDriveModel final : public MotionModel {
 public:
  const char* name() const override;
  Pose2D integrate(const Pose2D& pose, const Twist2D& cmd, double dt) const override;
  void sampleCommands(const Twist2D& current, const VelocityLimits& limits, const SampleSpec& spec,
                      double dt, std::vector<Twist2D>& out) const override;
  bool canRotateInPlace() const override;
};

std::unique_ptr<MotionModel> makeMotionModel(const std::string& type);   // "diff_drive"。未実装は std::invalid_argument

}  // namespace social_nav
