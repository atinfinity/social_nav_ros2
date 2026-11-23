#include "social_nav_core/crowd_analyzer.hpp"

#include <algorithm>
#include <cmath>

namespace social_nav {

namespace {
constexpr double kPi = 3.14159265358979323846;

struct Proj { double t; double dist2; double raw; };

// 線分 a-b への点 p の射影(t は [0,1] にクランプ)
Proj projectOnSegment(Vec2 a, Vec2 b, Vec2 p) {
  const double dx = b.x - a.x, dy = b.y - a.y;
  const double len2 = dx * dx + dy * dy;
  const double raw = len2 > 1e-12 ? ((p.x - a.x) * dx + (p.y - a.y) * dy) / len2 : 0.0;
  const double t = std::min(1.0, std::max(0.0, raw));
  const double qx = a.x + t * dx - p.x, qy = a.y + t * dy - p.y;
  return {t, qx * qx + qy * qy, raw};
}
}  // namespace

double crowdDensity(const TrackedPeople& people, Vec2 center, double radius) {
  if (radius <= 0.0) return 0.0;
  const double r2 = radius * radius;
  int n = 0;
  for (const auto& a : people.agents) {
    const double dx = a.pos.x - center.x, dy = a.pos.y - center.y;
    if (dx * dx + dy * dy <= r2) ++n;
  }
  return n / (kPi * r2);
}

double speedLimitRatio(double density, double low, double high, double min_ratio) {
  if (density <= low) return 1.0;
  if (density >= high || high <= low) return min_ratio;
  const double u = (density - low) / (high - low);
  return 1.0 - u * (1.0 - min_ratio);
}

std::vector<PersonOnPath> peopleOnPath(const std::vector<Pose2D>& path, const Pose2D& robot,
                                       const TrackedPeople& people, const CorridorParams& prm) {
  std::vector<PersonOnPath> out;
  if (path.size() < 2 || people.agents.empty()) return out;

  // ロボットに最も近い線分と、その射影点
  const Vec2 rp{robot.x, robot.y};
  std::size_t best = 0;
  Proj bp{0.0, 1e300, 0.0};
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const Proj pr = projectOnSegment({path[i].x, path[i].y}, {path[i + 1].x, path[i + 1].y}, rp);
    if (pr.dist2 < bp.dist2) { bp = pr; best = i; }
  }

  // 射影点から先の線分列(始点・終点・累積距離)
  struct Seg { Vec2 a, b; double s0, len; };
  std::vector<Seg> segs;
  double s = 0.0;
  // ロボットが path[0] より手前にいるときは「ロボット位置→path[0]」を先頭線分に足す
  if (best == 0 && bp.raw < 0.0) {
    const Vec2 a = rp;
    const Vec2 b{path[0].x, path[0].y};
    const double len = std::hypot(b.x - a.x, b.y - a.y);
    if (len >= 1e-9) {
      segs.push_back({a, b, s, len});
      s += len;
    }
  }
  for (std::size_t i = best; i + 1 < path.size() && s < prm.lookahead; ++i) {
    Vec2 a{path[i].x, path[i].y};
    const Vec2 b{path[i + 1].x, path[i + 1].y};
    if (i == best) {
      a = {a.x + bp.t * (b.x - a.x), a.y + bp.t * (b.y - a.y)};
    }
    const double len = std::hypot(b.x - a.x, b.y - a.y);
    if (len < 1e-9) continue;
    segs.push_back({a, b, s, len});
    s += len;
  }

  for (const auto& ag : people.agents) {
    double best_lat2 = 1e300, best_s = 0.0;
    const Seg* bs = nullptr;
    double best_raw = 0.0;
    for (const auto& sg : segs) {
      const Proj pr = projectOnSegment(sg.a, sg.b, ag.pos);
      if (pr.dist2 < best_lat2) {
        best_lat2 = pr.dist2; best_s = sg.s0 + pr.t * sg.len; bs = &sg; best_raw = pr.raw;
      }
    }
    // ロボット背後・真横(先頭線分の始点に張り付く人)は経路上とみなさない
    if (bs && bs == &segs.front() && best_raw <= 1e-9) continue;
    if (!bs || best_s > prm.lookahead) continue;
    if (best_lat2 > prm.half_width * prm.half_width) continue;
    const double tx = (bs->b.x - bs->a.x) / bs->len, ty = (bs->b.y - bs->a.y) / bs->len;
    PersonOnPath p;
    p.id = ag.id;
    p.dist_along = best_s;
    p.speed_along = ag.vel.x * tx + ag.vel.y * ty;
    p.speed = std::hypot(ag.vel.x, ag.vel.y);
    out.push_back(p);
  }
  std::sort(out.begin(), out.end(),
            [](const PersonOnPath& l, const PersonOnPath& r) { return l.dist_along < r.dist_along; });
  return out;
}

}  // namespace social_nav
