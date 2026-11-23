#include "sim/social_force.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace social_nav {
namespace sim {

Vec2 nearestPointOnSegment(Vec2 p, const Segment& s) {
  const double dx = s.b.x - s.a.x, dy = s.b.y - s.a.y;
  const double len2 = dx * dx + dy * dy;
  double t = 0.0;
  if (len2 > 1e-12) t = std::min(1.0, std::max(0.0, ((p.x - s.a.x) * dx + (p.y - s.a.y) * dy) / len2));
  return {s.a.x + t * dx, s.a.y + t * dy};
}

double distanceToWalls(Vec2 p, const std::vector<Segment>& walls) {
  double best = 1e9;
  for (const Segment& w : walls) {
    const Vec2 q = nearestPointOnSegment(p, w);
    best = std::min(best, std::hypot(p.x - q.x, p.y - q.y));
  }
  return best;
}

bool isRepeatedWallCorner(Vec2 p, const std::vector<Segment>& walls, std::size_t wi, Vec2 q) {
  // nearestPointOnSegment の t = 1 は a + 1.0 * (b - a) で、丸めにより b と一致しないことがあるので許容誤差で比べる
  constexpr double kTol = 1e-9;
  const auto same = [](Vec2 u, Vec2 v) { return std::abs(u.x - v.x) <= kTol && std::abs(u.y - v.y) <= kTol; };
  const Segment& w = walls[wi];
  if (!same(q, w.a) && !same(q, w.b)) return false;
  for (std::size_t wj = 0; wj < wi; ++wj)
    if (same(nearestPointOnSegment(p, walls[wj]), q)) return true;
  return false;
}

namespace {
double cross(Vec2 o, Vec2 a, Vec2 b) { return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x); }
int sgn(double v) { return v > 0.0 ? 1 : (v < 0.0 ? -1 : 0); }
bool onSegment(Vec2 a, Vec2 b, Vec2 p) {   // p は直線 ab 上にある前提
  return std::min(a.x, b.x) <= p.x && p.x <= std::max(a.x, b.x) && std::min(a.y, b.y) <= p.y && p.y <= std::max(a.y, b.y);
}
}  // namespace

bool segmentsIntersect(const Segment& s, const Segment& t) {
  const int d1 = sgn(cross(s.a, s.b, t.a)), d2 = sgn(cross(s.a, s.b, t.b));
  const int d3 = sgn(cross(t.a, t.b, s.a)), d4 = sgn(cross(t.a, t.b, s.b));
  if (d1 * d2 < 0 && d3 * d4 < 0) return true;
  if (d1 == 0 && onSegment(s.a, s.b, t.a)) return true;
  if (d2 == 0 && onSegment(s.a, s.b, t.b)) return true;
  if (d3 == 0 && onSegment(t.a, t.b, s.a)) return true;
  if (d4 == 0 && onSegment(t.a, t.b, s.b)) return true;
  return false;
}

bool lineOfSight(Vec2 from, Vec2 to, const std::vector<Segment>& walls) {
  const Segment ray{from, to};
  for (const Segment& w : walls)
    if (segmentsIntersect(ray, w)) return false;
  return true;
}

bool visibleAny(Vec2 from, Vec2 center, double radius, const std::vector<Segment>& walls) {
  if (lineOfSight(from, center, walls)) return true;
  const double dx = center.x - from.x, dy = center.y - from.y;
  const double d = std::hypot(dx, dy);
  if (d < 1e-9) return true;
  const double nx = -dy / d * radius, ny = dx / d * radius;
  return lineOfSight(from, {center.x + nx, center.y + ny}, walls) || lineOfSight(from, {center.x - nx, center.y - ny}, walls);
}

SocialForceWorld::SocialForceWorld(SfmParams prm, std::vector<Segment> walls)
    : p_(prm), walls_(std::move(walls)) {}

void SocialForceWorld::step(std::vector<Pedestrian>& peds, const RobotBody* robot, double dt) const {
  const std::size_t n = peds.size();
  std::vector<Vec2> acc(n);
  std::vector<Vec2> dir(n);

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
    const bool arrived = gd < 0.3;
    const Vec2 e = (gd > 1e-9 && !arrived) ? Vec2{gx / gd, gy / gd} : Vec2{0.0, 0.0};
    dir[i] = e;
    const double v_des = arrived ? 0.0 : a.v0;
    double ax = (v_des * e.x - a.vel.x) / p_.tau;
    double ay = (v_des * e.y - a.vel.y) / p_.tau;

    auto repel = [&](Vec2 other_pos, double r_sum, bool anisotropic) {
      const double dx = a.pos.x - other_pos.x, dy = a.pos.y - other_pos.y;
      const double d = std::hypot(dx, dy);
      const double nx = d > 1e-9 ? dx / d : 1.0, ny = d > 1e-9 ? dy / d : 0.0;
      double w = 1.0;
      if (anisotropic) {
        const double cos_phi = -(e.x * nx + e.y * ny);   // 相手が進行方向前方なら 1
        w = p_.lambda + (1.0 - p_.lambda) * (1.0 + cos_phi) / 2.0;
      }
      const double f = p_.A * std::exp((r_sum - d) / p_.B) * w + p_.k_contact * std::max(0.0, r_sum - d);
      ax += f * nx;
      ay += f * ny;
    };
    for (std::size_t j = 0; j < n; ++j) {
      if (j == i) continue;
      repel(peds[j].pos, a.radius + peds[j].radius, true);
    }
    if (robot) repel(robot->pos, a.radius + robot->radius, true);
    for (std::size_t wi = 0; wi < walls_.size(); ++wi) {
      const Vec2 q = nearestPointOnSegment(a.pos, walls_[wi]);
      if (isRepeatedWallCorner(a.pos, walls_, wi, q)) continue;   // 共有する角は 1 回だけ数える
      const double dx = a.pos.x - q.x, dy = a.pos.y - q.y;
      const double d = std::hypot(dx, dy);
      const double nx = d > 1e-9 ? dx / d : 0.0, ny = d > 1e-9 ? dy / d : 0.0;
      const double f = p_.A * std::exp((a.radius - d) / p_.B) + p_.k_contact * std::max(0.0, a.radius - d);
      ax += f * nx;
      ay += f * ny;
    }
    acc[i] = {ax, ay};
  }

  for (std::size_t i = 0; i < n; ++i) {
    Pedestrian& a = peds[i];
    if (a.frozen) continue;
    a.vel.x += acc[i].x * dt;
    a.vel.y += acc[i].y * dt;
    const double sp = std::hypot(a.vel.x, a.vel.y);
    if (sp > p_.max_speed) {
      a.vel.x *= p_.max_speed / sp;
      a.vel.y *= p_.max_speed / sp;
    }
    a.pos.x += a.vel.x * dt;
    a.pos.y += a.vel.y * dt;
  }
}

}  // namespace sim
}  // namespace social_nav
