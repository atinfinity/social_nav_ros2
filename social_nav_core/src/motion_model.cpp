#include "social_nav_core/motion_model.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace social_nav {

const char* DiffDriveModel::name() const { return "diff_drive"; }

bool DiffDriveModel::canRotateInPlace() const { return true; }

Pose2D DiffDriveModel::integrate(const Pose2D& pose, const Twist2D& cmd, double dt) const {
  Pose2D out = pose;
  const double th1 = pose.theta + cmd.wz * dt;
  if (std::fabs(cmd.wz) < 1e-9) {
    out.x += cmd.vx * std::cos(pose.theta) * dt;
    out.y += cmd.vx * std::sin(pose.theta) * dt;
  } else {
    out.x += cmd.vx / cmd.wz * (std::sin(th1) - std::sin(pose.theta));
    out.y += cmd.vx / cmd.wz * (std::cos(pose.theta) - std::cos(th1));
  }
  out.theta = th1;
  return out;
}

namespace {
// [lo, hi] を n 等分した点(両端含む)。n<=1 なら中点
void linspace(double lo, double hi, int n, std::vector<double>& out) {
  out.clear();
  if (n <= 1) {
    out.push_back(0.5 * (lo + hi));
    return;
  }
  for (int i = 0; i < n; ++i) out.push_back(lo + (hi - lo) * i / (n - 1));
}
}  // namespace

void DiffDriveModel::sampleCommands(const Twist2D& current, const VelocityLimits& limits,
                                    const SampleSpec& spec, double dt, std::vector<Twist2D>& out) const {
  double v_lo = std::max(limits.v_min, current.vx - limits.acc_v * dt);
  double v_hi = std::min(limits.v_max, current.vx + limits.acc_v * dt);
  double w_lo = std::max(-limits.w_max, current.wz - limits.acc_w * dt);
  double w_hi = std::min(limits.w_max, current.wz + limits.acc_w * dt);
  // 現在速度が範囲外すぎて窓が空になるときは、加速度制約を守ったまま範囲に最も近い 1 点にする
  if (v_lo > v_hi) {
    if (current.vx > limits.v_max) v_lo = v_hi = current.vx - limits.acc_v * dt;
    else v_lo = v_hi = current.vx + limits.acc_v * dt;
  }
  if (w_lo > w_hi) {
    if (current.wz > limits.w_max) w_lo = w_hi = current.wz - limits.acc_w * dt;
    else w_lo = w_hi = current.wz + limits.acc_w * dt;
  }
  std::vector<double> vs, ws;
  linspace(v_lo, v_hi, spec.v_samples, vs);
  linspace(w_lo, w_hi, spec.w_samples, ws);
  for (double v : vs) {
    for (double w : ws) {
      Twist2D t;
      t.vx = v;
      t.vy = 0.0;
      t.wz = w;
      out.push_back(t);
    }
  }
}

std::unique_ptr<MotionModel> makeMotionModel(const std::string& type) {
  if (type == "diff_drive") return std::make_unique<DiffDriveModel>();
  throw std::invalid_argument("makeMotionModel: unsupported motion model type: " + type);
}

}  // namespace social_nav
