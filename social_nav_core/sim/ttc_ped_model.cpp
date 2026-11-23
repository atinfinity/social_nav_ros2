#include "sim/ttc_ped_model.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace social_nav {
namespace sim {

TtcPedModel::TtcPedModel(TtcParams prm, std::vector<Segment> walls, uint32_t noise_seed)
    : p_(prm), walls_(std::move(walls)), eng_(noise_seed) {}

// sfm2 と同じ作り(mt19937 の生出力 2 個 -> Box-Muller)。人ごとに 2 個の標準正規
void TtcPedModel::normalPair(double& z0, double& z1) {
  const double kInv = 1.0 / 4294967296.0;
  const double u1 = (static_cast<double>(eng_()) + 0.5) * kInv;
  const double u2 = (static_cast<double>(eng_()) + 0.5) * kInv;
  const double r = std::sqrt(-2.0 * std::log(u1));
  const double th = 2.0 * M_PI * u2;
  z0 = r * std::cos(th);
  z1 = r * std::sin(th);
}

Vec2 TtcPedModel::interactionForce(Vec2 pa, Vec2 va, double ra, Vec2 pb, Vec2 vb, double rb) const {
  const double R = ra + rb;
  const Vec2 x{pb.x - pa.x, pb.y - pa.y};          // a から b への相対位置
  const Vec2 v{va.x - vb.x, va.y - vb.y};          // 相対速度(a が b へ近づく向きが正)
  const double a = v.x * v.x + v.y * v.y;
  const double b = x.x * v.x + x.y * v.y;
  const double c = x.x * x.x + x.y * x.y - R * R;
  if (a < 1e-12) return {0.0, 0.0};
  if (c <= 0.0) return {0.0, 0.0};                  // 重なり: 相互作用項は 0。接触ばね(step 側)だけが効く
  if (b <= 0.0) return {0.0, 0.0};                  // 離れていく: tau = 無限大
  const double disc = b * b - a * c;
  if (disc <= 0.0) return {0.0, 0.0};               // 外形がかすらない: tau = 無限大
  const double s = std::sqrt(std::max(disc, 1e-12));
  const double tau = (b - s) / a;
  // F_a = -grad_{x_a} E = dE/dtau * dtau/dx(x = x_b - x_a)
  const double dEdtau = -p_.k * std::exp(-tau / p_.tau0) / (tau * tau * tau) * (2.0 + tau / p_.tau0);
  const Vec2 g{(v.x - (b * v.x - a * x.x) / s) / a, (v.y - (b * v.y - a * x.y) / s) / a};
  Vec2 f{dEdtau * g.x, dEdtau * g.y};
  const double fm = std::hypot(f.x, f.y);
  if (fm > p_.f_max) { f.x *= p_.f_max / fm; f.y *= p_.f_max / fm; }
  return f;
}

void TtcPedModel::step(std::vector<Pedestrian>& peds, const RobotBody* robot, double dt) {
  const PedParams& P = p_.base;
  const std::size_t n = peds.size();
  if (noise_.size() != n) noise_.assign(n, Vec2{});

  const double decay = std::exp(-dt / P.noise_tau);
  const double inj = P.noise_sigma * std::sqrt(1.0 - decay * decay);
  for (std::size_t i = 0; i < n; ++i) {
    double z0, z1;
    normalPair(z0, z1);
    noise_[i].x = noise_[i].x * decay + inj * z0;
    noise_[i].y = noise_[i].y * decay + inj * z1;
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
    if (gd < 0.3) { stop[i] = 1; continue; }
    const Vec2 e{gx / gd, gy / gd};
    double ax = (a.v0 * e.x - a.vel.x) / P.tau + noise_[i].x;
    double ay = (a.v0 * e.y - a.vel.y) / P.tau + noise_[i].y;

    auto contact = [&](Vec2 op, double r_sum) {
      const double dx = a.pos.x - op.x, dy = a.pos.y - op.y;
      const double d = std::hypot(dx, dy);
      if (d >= r_sum || d < 1e-9) return;
      const double f = P.k_contact * (r_sum - d);
      ax += f * dx / d;
      ay += f * dy / d;
    };
    for (std::size_t j = 0; j < n; ++j) {
      if (j == i) continue;
      const Vec2 f = interactionForce(a.pos, a.vel, a.radius, peds[j].pos, peds[j].vel, peds[j].radius);
      ax += f.x;
      ay += f.y;
      contact(peds[j].pos, a.radius + peds[j].radius);
    }
    if (robot) {
      const Vec2 f = interactionForce(a.pos, a.vel, a.radius, robot->pos, robot->vel, robot->radius);
      ax += f.x;
      ay += f.y;
      contact(robot->pos, a.radius + robot->radius);
    }
    for (std::size_t wi = 0; wi < walls_.size(); ++wi) {   // sfm2 の壁の扱いと同じ
      const Vec2 q = nearestPointOnSegment(a.pos, walls_[wi]);
      if (isRepeatedWallCorner(a.pos, walls_, wi, q)) continue;   // 共有する角は 1 回だけ数える
      const double dx = a.pos.x - q.x, dy = a.pos.y - q.y;
      const double d = std::hypot(dx, dy);
      if (d < 1e-9) continue;
      const double f = P.A_wall * std::exp((a.radius - d) / P.B_wall) + P.k_contact * std::max(0.0, a.radius - d);
      ax += f * dx / d;
      ay += f * dy / d;
    }
    acc[i] = {ax, ay};
  }

  for (std::size_t i = 0; i < n; ++i) {
    Pedestrian& a = peds[i];
    if (a.frozen) continue;
    if (stop[i]) { a.vel = {0.0, 0.0}; continue; }
    a.vel.x += acc[i].x * dt;
    a.vel.y += acc[i].y * dt;
    const double sp = std::hypot(a.vel.x, a.vel.y);
    const double cap = std::min(P.max_speed, P.speed_cap * a.v0);
    if (sp > cap) { a.vel.x *= cap / sp; a.vel.y *= cap / sp; }
    a.pos.x += a.vel.x * dt;
    a.pos.y += a.vel.y * dt;
  }
}

}  // namespace sim
}  // namespace social_nav
