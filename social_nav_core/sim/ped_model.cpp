#include "sim/ped_model.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <utility>

namespace social_nav {
namespace sim {

PedModel::PedModel(PedParams prm, std::vector<Segment> walls, uint32_t noise_seed)
    : p_(prm), walls_(std::move(walls)), eng_(noise_seed) {}

// 一様(0,1) は mt19937 の 32 bit 出力 x から (x + 0.5) / 2^32。std の distribution は使わない(実装間で値が違うため)。
void PedModel::normalPair(double& z0, double& z1) {
  const double kInv = 1.0 / 4294967296.0;
  const double u1 = (static_cast<double>(eng_()) + 0.5) * kInv;
  const double u2 = (static_cast<double>(eng_()) + 0.5) * kInv;
  const double r = std::sqrt(-2.0 * std::log(u1));
  const double th = 2.0 * M_PI * u2;
  z0 = r * std::cos(th);
  z1 = r * std::sin(th);
}

Vec2 PedModel::interactionForce(Vec2 pa, Vec2 va, Vec2 pb, Vec2 vb) const {
  const double dx = pb.x - pa.x, dy = pb.y - pa.y;
  const double d = std::hypot(dx, dy);
  if (d < 1e-9 || d > p_.range) return {0.0, 0.0};
  const Vec2 e{dx / d, dy / d};                       // a から b へ
  const Vec2 D{p_.lambda * (va.x - vb.x) + e.x, p_.lambda * (va.y - vb.y) + e.y};
  const double Dn = std::hypot(D.x, D.y);
  if (Dn < 1e-9) return {0.0, 0.0};
  const Vec2 t{D.x / Dn, D.y / Dn};
  const Vec2 nl{-t.y, t.x};                           // t の左法線
  const double theta = std::atan2(t.x * e.y - t.y * e.x, t.x * e.x + t.y * e.y);   // t から e への角
  const double K = theta >= 0.0 ? 1.0 : -1.0;         // 正面一致(theta=0)は +1 に倒して対称性を破る
  const double B = p_.gamma * Dn;
  const double mag = p_.A * std::exp(-d / B);
  const double ft = std::exp(-(p_.n_prime * B * theta) * (p_.n_prime * B * theta));
  const double fn = std::exp(-(p_.n * B * theta) * (p_.n * B * theta));
  // f = -A exp(-d/B) [ exp(-(n' B theta)^2) t + exp(-(n B theta)^2) K n ]
  return {-mag * (ft * t.x + fn * K * nl.x), -mag * (ft * t.y + fn * K * nl.y)};
}

// group_id >= kPaperGazeGroupIdMin の視線(向きのみ Moussaid 2010 の f = -beta alpha V。係数と視野角は論文と異なる)。重心が視野の外なら速度に沿って減速する。
// 既存の式(crowd 用)とは別関数にして、その浮動小数点式の並びに触れない。
static void paperGaze(const PedParams& p, Vec2 vel, double cx, double cy, double& ax, double& ay) {
  const double sp = std::hypot(vel.x, vel.y);
  if (sp <= 1e-3) return;
  const double alpha = std::atan2(vel.x * cy - vel.y * cx, vel.x * cx + vel.y * cy);
  const double half = 0.5 * p.group_fov_deg * M_PI / 180.0;
  if (std::fabs(alpha) > half) {
    const double f = p.group_vis_gain_paper * (std::fabs(alpha) - half);
    ax -= f * vel.x;
    ay -= f * vel.y;
  }
}

// group_id >= kPaperGazeGroupIdMin の群れの同群メンバー間の相互作用から、横(法線)成分 K を除く(横並びの 2 人を K が逆向きに押して隊形が崩れるため)。
// interactionForce と同じ式で fn の項だけ 0(crowd 用の既存の関数は触らない)
static Vec2 interactionForceNoK(const PedParams& p_, Vec2 pa, Vec2 va, Vec2 pb, Vec2 vb) {
  const double dx = pb.x - pa.x, dy = pb.y - pa.y;
  const double d = std::hypot(dx, dy);
  if (d < 1e-9 || d > p_.range) return {0.0, 0.0};
  const Vec2 e{dx / d, dy / d};
  const Vec2 D{p_.lambda * (va.x - vb.x) + e.x, p_.lambda * (va.y - vb.y) + e.y};
  const double Dn = std::hypot(D.x, D.y);
  if (Dn < 1e-9) return {0.0, 0.0};
  const Vec2 t{D.x / Dn, D.y / Dn};
  const double theta = std::atan2(t.x * e.y - t.y * e.x, t.x * e.x + t.y * e.y);
  const double B = p_.gamma * Dn;
  const double mag = p_.A * std::exp(-d / B);
  const double ft = std::exp(-(p_.n_prime * B * theta) * (p_.n_prime * B * theta));
  return {-mag * (ft * t.x), -mag * (ft * t.y)};
}

void PedModel::step(std::vector<Pedestrian>& peds, const RobotBody* robot, double dt) {
  const std::size_t n = peds.size();
  if (noise_.size() != n) noise_.assign(n, Vec2{});

  // 乱数は frozen / 到達済みでも同数引く(人数・順序だけで消費数が決まる)
  const double decay = std::exp(-dt / p_.noise_tau);
  const double inj = p_.noise_sigma * std::sqrt(1.0 - decay * decay);
  for (std::size_t i = 0; i < n; ++i) {
    double z0, z1;
    normalPair(z0, z1);
    noise_[i].x = noise_[i].x * decay + inj * z0;
    noise_[i].y = noise_[i].y * decay + inj * z1;
  }

  struct GroupSum { double x{0}, y{0}; int n{0}; };
  std::map<int, GroupSum> groups;
  for (const Pedestrian& q : peds) {
    if (q.group_id < 0) continue;
    GroupSum& g = groups[q.group_id];
    g.x += q.pos.x; g.y += q.pos.y; ++g.n;
  }

  std::vector<Vec2> acc(n);
  std::vector<char> stop(n, 0);
  for (std::size_t i = 0; i < n; ++i) {
    Pedestrian& a = peds[i];
    if (a.frozen) continue;
    double gx = a.goal.x - a.pos.x, gy = a.goal.y - a.pos.y;
    double gd = std::hypot(gx, gy);
    if (a.loop && gd < 0.5) {
      std::swap(a.goal, a.home);
      gx = a.goal.x - a.pos.x;
      gy = a.goal.y - a.pos.y;
      gd = std::hypot(gx, gy);
    }
    if (gd < 0.3) {   // 到達後は停止(斥力で漂わない)
      stop[i] = 1;
      continue;
    }
    const Vec2 e{gx / gd, gy / gd};
    const bool phone = a.kind == 2;
    const double v0 = phone ? a.v0 * p_.phone_v0_scale : a.v0;
    const double att = phone ? a.attention * p_.phone_attention : a.attention;
    const double nz = phone ? p_.phone_noise_scale : 1.0;
    double ax = (v0 * e.x - a.vel.x) / p_.tau + nz * noise_[i].x;
    double ay = (v0 * e.y - a.vel.y) / p_.tau + nz * noise_[i].y;

    auto contact = [&](Vec2 op, double r_sum) {
      const double dx = a.pos.x - op.x, dy = a.pos.y - op.y;
      const double d = std::hypot(dx, dy);
      if (d >= r_sum || d < 1e-9) return;
      const double f = p_.k_contact * (r_sum - d);
      ax += f * dx / d;
      ay += f * dy / d;
    };
    const double cos_half_fov = std::cos(0.5 * p_.fov_deg * M_PI / 180.0);
    // 視野内なら 1、視野外なら rear_weight。判定は目標方向 e(速度ではない)
    auto fovW = [&](Vec2 op) {
      const double dx = op.x - a.pos.x, dy = op.y - a.pos.y;
      const double d = std::hypot(dx, dy);
      if (d < 1e-9) return 1.0;
      return (e.x * dx + e.y * dy) / d >= cos_half_fov ? 1.0 : p_.rear_weight;
    };
    for (std::size_t j = 0; j < n; ++j) {
      if (j == i) continue;
      const bool same_group = a.group_id >= 0 && a.group_id == peds[j].group_id;
      const Vec2 f = (same_group && a.group_id >= kPaperGazeGroupIdMin)
                         ? interactionForceNoK(p_, a.pos, a.vel, peds[j].pos, peds[j].vel)
                         : interactionForce(a.pos, a.vel, peds[j].pos, peds[j].vel);
      const double w = fovW(peds[j].pos) * ((a.group_id >= 0 && a.group_id == peds[j].group_id) ? p_.group_inter_scale : 1.0);
      ax += w * att * f.x;
      ay += w * att * f.y;
      contact(peds[j].pos, a.radius + peds[j].radius);
    }
    if (robot) {
      const Vec2 f = interactionForce(a.pos, a.vel, robot->pos, robot->vel);
      const double w = fovW(robot->pos);
      ax += w * att * f.x;
      ay += w * att * f.y;
      contact(robot->pos, a.radius + robot->radius);
    }
    if (a.group_id >= 0) {
      const GroupSum& g = groups[a.group_id];
      if (g.n >= 2) {
        const Vec2 cm{g.x / g.n, g.y / g.n};
        const double cx = cm.x - a.pos.x, cy = cm.y - a.pos.y;
        const double cd = std::hypot(cx, cy);
        const double sp = std::hypot(a.vel.x, a.vel.y);
        if (cd > 1e-9) {
          // 視線: 重心が視野(進行方向 +-fov/2)の外なら、group_id >= kPaperGazeGroupIdMin は減速、それ以外は重心側へ向ける横加速度
          if (a.group_id >= kPaperGazeGroupIdMin) {
            paperGaze(p_, a.vel, cx, cy, ax, ay);
          } else if (sp > 1e-3) {
            const double alpha = std::atan2(a.vel.x * cy - a.vel.y * cx, a.vel.x * cx + a.vel.y * cy);  // 進行方向から重心への角
            const double half = 0.5 * p_.group_fov_deg * M_PI / 180.0;
            if (std::fabs(alpha) > half) {
              const double s = alpha > 0 ? 1.0 : -1.0;
              const double f = p_.group_vis_gain * (std::fabs(alpha) - half) * sp;
              ax += s * f * (-a.vel.y / sp);
              ay += s * f * (a.vel.x / sp);
            }
          }
          // 凝集: 重心が閾値より遠ければ重心へ
          const double thr = p_.group_dist_free * 0.5 * (g.n - 1) + p_.group_dist_base;
          if (cd > thr) {
            ax += p_.group_attract * (cd - thr) * cx / cd;
            ay += p_.group_attract * (cd - thr) * cy / cd;
          }
        }
        // 群内斥力: 近すぎる仲間から離れる(線形。体の接触ばねより手前で効く)
        for (std::size_t j = 0; j < n; ++j) {
          if (j == i || peds[j].group_id != a.group_id) continue;
          const double dx = a.pos.x - peds[j].pos.x, dy = a.pos.y - peds[j].pos.y;
          const double d = std::hypot(dx, dy);
          if (d < 1e-9 || d >= p_.group_repulse_dist) continue;
          const double f = p_.group_repulse * (p_.group_repulse_dist - d) / p_.group_repulse_dist;
          ax += f * dx / d;
          ay += f * dy / d;
        }
      }
    }
    for (std::size_t wi = 0; wi < walls_.size(); ++wi) {
      const Vec2 q = nearestPointOnSegment(a.pos, walls_[wi]);
      if (isRepeatedWallCorner(a.pos, walls_, wi, q)) continue;   // 共有する角は 1 回だけ数える
      const double dx = a.pos.x - q.x, dy = a.pos.y - q.y;
      const double d = std::hypot(dx, dy);
      if (d < 1e-9) continue;
      const double f = p_.A_wall * std::exp((a.radius - d) / p_.B_wall) + p_.k_contact * std::max(0.0, a.radius - d);
      ax += f * dx / d;
      ay += f * dy / d;
    }
    acc[i] = {ax, ay};
  }

  for (std::size_t i = 0; i < n; ++i) {
    Pedestrian& a = peds[i];
    if (a.frozen) continue;
    if (stop[i]) {
      a.vel = {0.0, 0.0};
      continue;
    }
    a.vel.x += acc[i].x * dt;
    a.vel.y += acc[i].y * dt;
    const double sp = std::hypot(a.vel.x, a.vel.y);
    const double v0_eff = a.kind == 2 ? a.v0 * p_.phone_v0_scale : a.v0;
    const double cap = std::min(p_.max_speed, p_.speed_cap * v0_eff);
    if (sp > cap) {
      a.vel.x *= cap / sp;
      a.vel.y *= cap / sp;
    }
    a.pos.x += a.vel.x * dt;
    a.pos.y += a.vel.y * dt;
  }
}

}  // namespace sim
}  // namespace social_nav
