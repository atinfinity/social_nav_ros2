// Portions of this file are based on src/Agent.cc of the RVO2 Library
// (https://github.com/snape/RVO2, commit 6b4eb00):
//   SPDX-FileCopyrightText: 2008 University of North Carolina at Chapel Hill
//   SPDX-License-Identifier: Apache-2.0
// Modified for social_nav: rewritten for this simulator's types, double
// precision, walls as independent zero-width segments in the given order,
// parameters in OrcaParams, a variable avoidance share instead of a fixed 0.5,
// and an exhaustive neighbor search instead of a k-d tree.

#include "sim/orca_ped_model.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace social_nav {
namespace sim {

namespace {

constexpr double kEps = 1e-9;
constexpr double kInf = std::numeric_limits<double>::infinity();

struct Line { Vec2 point, dir; };   // dir は単位ベクトル。許容側は dir の左

double dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }
double det(Vec2 a, Vec2 b) { return a.x * b.y - a.y * b.x; }
double sq(Vec2 a) { return dot(a, a); }
Vec2 sub(Vec2 a, Vec2 b) { return {a.x - b.x, a.y - b.y}; }
Vec2 add(Vec2 a, Vec2 b) { return {a.x + b.x, a.y + b.y}; }
Vec2 mul(Vec2 a, double s) { return {a.x * s, a.y * s}; }
Vec2 unit(Vec2 a) {
  const double n = std::sqrt(sq(a));
  return n > kEps ? Vec2{a.x / n, a.y / n} : Vec2{1.0, 0.0};
}
Vec2 perpL(Vec2 a) { return {-a.y, a.x}; }

// 直線 lines[lineNo] 上で、lines[0..lineNo) と半径 radius の円を満たす最適点を求める
bool lp1(const std::vector<Line>& lines, std::size_t lineNo, double radius, Vec2 opt, bool direction_opt, Vec2& result) {
  const Line& L = lines[lineNo];
  const double dp = dot(L.point, L.dir);
  const double disc = dp * dp + radius * radius - sq(L.point);
  if (disc < 0.0) return false;
  const double sd = std::sqrt(disc);
  double t_left = -dp - sd, t_right = -dp + sd;
  for (std::size_t i = 0; i < lineNo; ++i) {
    const double denom = det(L.dir, lines[i].dir);
    const double numer = det(lines[i].dir, sub(L.point, lines[i].point));
    if (std::fabs(denom) <= kEps) {
      if (numer < 0.0) return false;
      continue;
    }
    const double t = numer / denom;
    if (denom >= 0.0) t_right = std::min(t_right, t);
    else t_left = std::max(t_left, t);
    if (t_left > t_right) return false;
  }
  if (direction_opt) {
    result = add(L.point, mul(L.dir, dot(opt, L.dir) > 0.0 ? t_right : t_left));
  } else {
    const double t = std::clamp(dot(L.dir, sub(opt, L.point)), t_left, t_right);
    result = add(L.point, mul(L.dir, t));
  }
  return true;
}

// 満たせない最初の線の番号を返す(全部満たせれば lines.size())
std::size_t lp2(const std::vector<Line>& lines, double radius, Vec2 opt, bool direction_opt, Vec2& result) {
  if (direction_opt) result = mul(opt, radius);
  else if (sq(opt) > radius * radius) result = mul(unit(opt), radius);
  else result = opt;
  for (std::size_t i = 0; i < lines.size(); ++i) {
    if (det(lines[i].dir, sub(lines[i].point, result)) > 0.0) {
      const Vec2 keep = result;
      if (!lp1(lines, i, radius, opt, direction_opt, result)) {
        result = keep;
        return i;
      }
    }
  }
  return lines.size();
}

// 実行不能のとき: 壁の線(先頭 n_obst 本)は守り、他の線への侵入の最大値を最小にする
void lp3(const std::vector<Line>& lines, std::size_t n_obst, std::size_t begin, double radius, Vec2& result) {
  double distance = 0.0;
  for (std::size_t i = begin; i < lines.size(); ++i) {
    if (det(lines[i].dir, sub(lines[i].point, result)) > distance) {
      std::vector<Line> proj(lines.begin(), lines.begin() + static_cast<std::ptrdiff_t>(n_obst));
      for (std::size_t j = n_obst; j < i; ++j) {
        Line l;
        const double d = det(lines[i].dir, lines[j].dir);
        if (std::fabs(d) <= kEps) {
          if (dot(lines[i].dir, lines[j].dir) > 0.0) continue;
          l.point = mul(add(lines[i].point, lines[j].point), 0.5);
        } else {
          l.point = add(lines[i].point, mul(lines[i].dir, det(lines[j].dir, sub(lines[i].point, lines[j].point)) / d));
        }
        l.dir = unit(sub(lines[j].dir, lines[i].dir));
        proj.push_back(l);
      }
      const Vec2 keep = result;
      if (lp2(proj, radius, perpL(lines[i].dir), true, result) < proj.size()) result = keep;
      distance = det(lines[i].dir, sub(lines[i].point, result));
    }
  }
}

// 線分(幅 0 の壁)に対する ORCA 線。両端は丸い端(凸の頂点)として扱い、線分そのものが脚になりうる。
// 戻り値: 線を作ったら true。already: 既存の ORCA 線で既に覆われている場合は作らない(呼び出し側で判定)
void wallLines(Vec2 pos, Vec2 vel, double radius, double inv_t, Vec2 w1, Vec2 w2, std::vector<Line>& lines) {
  // 自分が辺の右側になる向きにそろえる(辺の左が壁の内側、右が自由空間)
  if (det(sub(w2, w1), sub(pos, w1)) > 0.0) std::swap(w1, w2);
  const Vec2 rel1 = sub(w1, pos), rel2 = sub(w2, pos);
  const double r2 = radius * radius;

  for (const Line& l : lines) {   // 既に覆われているか
    if (det(sub(mul(rel1, inv_t), l.point), l.dir) - inv_t * radius >= -kEps &&
        det(sub(mul(rel2, inv_t), l.point), l.dir) - inv_t * radius >= -kEps)
      return;
  }

  const double d1 = sq(rel1), d2 = sq(rel2);
  const Vec2 ov = sub(w2, w1);
  const double ov2 = sq(ov);
  if (ov2 < kEps) return;
  const double s = dot(mul(rel1, -1.0), ov) / ov2;
  const double dist_line2 = sq(sub(mul(rel1, -1.0), mul(ov, s)));
  const Vec2 od = unit(ov);   // 辺の向き(w1 から w2)

  Line line;
  if (s < 0.0 && d1 <= r2) {   // 左端と重なっている
    line.point = {0.0, 0.0};
    line.dir = unit({-rel1.y, rel1.x});
    lines.push_back(line);
    return;
  }
  if (s > 1.0 && d2 <= r2) {   // 右端と重なっている
    line.point = {0.0, 0.0};
    line.dir = unit({-rel2.y, rel2.x});
    lines.push_back(line);
    return;
  }
  if (s >= 0.0 && s <= 1.0 && dist_line2 <= r2) {   // 辺と重なっている
    line.point = {0.0, 0.0};
    line.dir = mul(od, -1.0);
    lines.push_back(line);
    return;
  }

  bool same = false;   // 片方の端だけで障害物が決まる(斜めから見る)
  Vec2 left_leg, right_leg;
  auto legs1 = [&](Vec2 rel, double d, Vec2& l, Vec2& r) {
    const double leg = std::sqrt(d - r2);
    l = {(rel.x * leg - rel.y * radius) / d, (rel.x * radius + rel.y * leg) / d};
    r = {(rel.x * leg + rel.y * radius) / d, (-rel.x * radius + rel.y * leg) / d};
  };
  Vec2 c1 = w1, c2 = w2;   // 障害物の左端・右端(世界座標)
  if (s < 0.0 && dist_line2 <= r2) {
    same = true;
    c2 = c1;
    legs1(rel1, d1, left_leg, right_leg);
  } else if (s > 1.0 && dist_line2 <= r2) {
    same = true;
    c1 = c2;
    legs1(rel2, d2, left_leg, right_leg);
  } else {
    Vec2 tmp;
    legs1(rel1, d1, left_leg, tmp);
    legs1(rel2, d2, tmp, right_leg);
  }

  // 幅 0 の壁なので、端の隣の辺は同じ線分(向きだけ逆)。脚が壁より内側に入るときは壁そのものを脚にする(foreign)。
  // 左脚は前の辺(w2 から w1 向き)を逆にした向き、右脚は次の辺の向き。通常は左 +od、右 -od。
  // 左端だけで決まる(s < 0)ときは両方 +od、右端だけで決まる(s > 1)ときは両方 -od。
  const Vec2 left_edge = (same && c1.x == w2.x && c1.y == w2.y) ? mul(od, -1.0) : od;
  const Vec2 right_edge = (same && c1.x == w1.x && c1.y == w1.y) ? od : mul(od, -1.0);
  bool left_foreign = false, right_foreign = false;
  if (det(left_leg, left_edge) >= 0.0) { left_leg = left_edge; left_foreign = true; }
  if (det(right_leg, right_edge) <= 0.0) { right_leg = right_edge; right_foreign = true; }

  const Vec2 left_cut = mul(sub(c1, pos), inv_t), right_cut = mul(sub(c2, pos), inv_t);
  const Vec2 cut_vec = sub(right_cut, left_cut);
  const double t = same ? 0.5 : dot(sub(vel, left_cut), cut_vec) / sq(cut_vec);
  const double t_left = dot(sub(vel, left_cut), left_leg);
  const double t_right = dot(sub(vel, right_cut), right_leg);

  if ((t < 0.0 && t_left < 0.0) || (same && t_left < 0.0 && t_right < 0.0)) {
    const Vec2 uw = unit(sub(vel, left_cut));
    line.dir = {uw.y, -uw.x};
    line.point = add(left_cut, mul(uw, radius * inv_t));
    lines.push_back(line);
    return;
  }
  if (t > 1.0 && t_right < 0.0) {
    const Vec2 uw = unit(sub(vel, right_cut));
    line.dir = {uw.y, -uw.x};
    line.point = add(right_cut, mul(uw, radius * inv_t));
    lines.push_back(line);
    return;
  }
  const double dc = (t < 0.0 || t > 1.0 || same) ? kInf : sq(sub(vel, add(left_cut, mul(cut_vec, t))));
  const double dl = t_left < 0.0 ? kInf : sq(sub(vel, add(left_cut, mul(left_leg, t_left))));
  const double dr = t_right < 0.0 ? kInf : sq(sub(vel, add(right_cut, mul(right_leg, t_right))));
  if (dc <= dl && dc <= dr) {
    line.dir = mul(od, -1.0);
    line.point = add(left_cut, mul(perpL(line.dir), radius * inv_t));
    lines.push_back(line);
    return;
  }
  if (dl <= dr) {
    if (left_foreign) return;
    line.dir = left_leg;
    line.point = add(left_cut, mul(perpL(line.dir), radius * inv_t));
    lines.push_back(line);
    return;
  }
  if (right_foreign) return;
  line.dir = mul(right_leg, -1.0);
  line.point = add(right_cut, mul(perpL(line.dir), radius * inv_t));
  lines.push_back(line);
}

struct Other { Vec2 pos, vel; double radius; double resp; double d2; };

// 他者 1 人に対する ORCA 線。resp は自分が負う回避の割合
Line agentLine(Vec2 pos, Vec2 vel, double radius, const Other& o, double inv_t, double dt) {
  const Vec2 rel = sub(o.pos, pos);
  const Vec2 rv = sub(vel, o.vel);
  const double d2 = sq(rel);
  const double cr = radius + o.radius, cr2 = cr * cr;
  Line line;
  Vec2 u;
  if (d2 > cr2) {
    const Vec2 w = sub(rv, mul(rel, inv_t));
    const double w2 = sq(w);
    const double dp1 = dot(w, rel);
    if (dp1 < 0.0 && dp1 * dp1 > cr2 * w2) {   // 切り口の円に射影
      const double wl = std::sqrt(w2);
      const Vec2 uw{w.x / wl, w.y / wl};
      line.dir = {uw.y, -uw.x};
      u = mul(uw, cr * inv_t - wl);
    } else {                                    // 円錐の脚に射影
      const double leg = std::sqrt(d2 - cr2);
      if (det(rel, w) > 0.0) line.dir = {(rel.x * leg - rel.y * cr) / d2, (rel.x * cr + rel.y * leg) / d2};
      else line.dir = {-(rel.x * leg + rel.y * cr) / d2, -(-rel.x * cr + rel.y * leg) / d2};
      u = sub(mul(line.dir, dot(rv, line.dir)), rv);
    }
  } else {                                      // 重なり: 1 刻みで解消する速度に射影
    const double it = 1.0 / dt;
    const Vec2 w = sub(rv, mul(rel, it));
    const double wl = std::sqrt(sq(w));
    const Vec2 uw = wl > kEps ? Vec2{w.x / wl, w.y / wl} : unit(mul(rel, -1.0));
    line.dir = {uw.y, -uw.x};
    u = mul(uw, cr * it - wl);
  }
  line.point = add(vel, mul(u, o.resp));
  return line;
}

}  // namespace

OrcaPedModel::OrcaPedModel(OrcaParams prm, std::vector<Segment> walls) : p_(prm), walls_(std::move(walls)) {}

void OrcaPedModel::step(std::vector<Pedestrian>& peds, const RobotBody* robot, double dt) const {
  const std::size_t n = peds.size();
  std::vector<Vec2> pref(n);
  std::vector<char> stop(n, 0), moving(n, 0);
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
    if (gd < p_.arrive_dist) { stop[i] = 1; continue; }
    moving[i] = 1;
    pref[i] = {a.v0 * gx / gd, a.v0 * gy / gd};
  }

  std::vector<Vec2> next(n);
  const double nd2 = p_.neighbor_dist * p_.neighbor_dist;
  std::vector<Other> cand;
  std::vector<Line> lines;
  for (std::size_t i = 0; i < n; ++i) {
    if (!moving[i]) continue;
    const Pedestrian& a = peds[i];
    const double max_speed = p_.speed_cap * a.v0;

    cand.clear();
    for (std::size_t j = 0; j < n; ++j) {
      if (j == i) continue;
      const double d2 = sq(sub(peds[j].pos, a.pos));
      if (d2 >= nd2) continue;
      // 動かない人(frozen、到達して停止)は避けられないので、歩行者側が全量を避ける
      cand.push_back({peds[j].pos, peds[j].vel, peds[j].radius, moving[j] ? p_.ped_responsibility : p_.robot_responsibility, d2});
    }
    if (robot) {
      const double d2 = sq(sub(robot->pos, a.pos));
      if (d2 < nd2) cand.push_back({robot->pos, robot->vel, robot->radius, p_.robot_responsibility, d2});
    }
    std::stable_sort(cand.begin(), cand.end(), [](const Other& x, const Other& y) { return x.d2 < y.d2; });
    if (static_cast<int>(cand.size()) > p_.max_neighbors) cand.resize(static_cast<std::size_t>(p_.max_neighbors));

    lines.clear();
    const double inv_to = 1.0 / p_.time_horizon_obst;
    const double obst_range = p_.time_horizon_obst * max_speed + a.radius;
    for (const Segment& w : walls_) {
      const Vec2 q = nearestPointOnSegment(a.pos, w);
      if (sq(sub(q, a.pos)) >= obst_range * obst_range) continue;
      wallLines(a.pos, a.vel, a.radius, inv_to, w.a, w.b, lines);
    }
    const std::size_t n_obst = lines.size();
    const double inv_t = 1.0 / p_.time_horizon;
    for (const Other& o : cand) lines.push_back(agentLine(a.pos, a.vel, a.radius, o, inv_t, dt));

    Vec2 v;
    const std::size_t fail = lp2(lines, max_speed, pref[i], false, v);
    if (fail < lines.size()) lp3(lines, n_obst, fail, max_speed, v);
    next[i] = v;
  }

  for (std::size_t i = 0; i < n; ++i) {
    Pedestrian& a = peds[i];
    if (a.frozen) continue;
    if (stop[i]) { a.vel = {0.0, 0.0}; continue; }
    a.vel = next[i];
    a.pos.x += a.vel.x * dt;
    a.pos.y += a.vel.y * dt;
  }
}

}  // namespace sim
}  // namespace social_nav
