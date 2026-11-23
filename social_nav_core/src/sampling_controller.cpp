#include "social_nav_core/sampling_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace social_nav {

namespace {

double distToSegment(double px, double py, const Pose2D& a, const Pose2D& b) {
  const double dx = b.x - a.x, dy = b.y - a.y;
  const double len2 = dx * dx + dy * dy;
  double t = 0.0;
  if (len2 > 1e-12) t = std::min(1.0, std::max(0.0, ((px - a.x) * dx + (py - a.y) * dy) / len2));
  return std::hypot(px - (a.x + t * dx), py - (a.y + t * dy));
}

double distToPath(double px, double py, const std::vector<Pose2D>& path) {
  if (path.size() == 1) return std::hypot(px - path[0].x, py - path[0].y);
  double best = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i + 1 < path.size(); ++i)
    best = std::min(best, distToSegment(px, py, path[i], path[i + 1]));
  return best;
}

// door_yield: 経路上でロボットの最近接点から前方 lookahead 以内に、障害物コストが c_narrow 以上になる最初の点を探す。
// 返り値はその点までの経路距離(なければ -1)。tangent は最近接区間の向き(単位ベクトル)
double findNarrowPoint(const Pose2D& pose, const std::vector<Pose2D>& path, const ObstacleCostFn& obstacles,
                       double c_narrow, double lookahead, Vec2* point, Vec2* tangent) {
  if (!obstacles || path.size() < 2) return -1.0;
  std::size_t bi = 0;
  double bd = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const double d = distToSegment(pose.x, pose.y, path[i], path[i + 1]);
    if (d < bd) { bd = d; bi = i; }
  }
  const double step = 0.05;
  double walked = 0.0;   // 最近接点からの経路距離
  bool first = true;
  for (std::size_t i = bi; i + 1 < path.size() && walked <= lookahead; ++i) {
    const double dx = path[i + 1].x - path[i].x, dy = path[i + 1].y - path[i].y;
    const double len = std::hypot(dx, dy);
    if (len < 1e-9) continue;
    const double ux = dx / len, uy = dy / len;
    if (first) { *tangent = {ux, uy}; }
    double t0 = 0.0;
    if (first) t0 = std::min(len, std::max(0.0, (pose.x - path[i].x) * ux + (pose.y - path[i].y) * uy));
    first = false;
    for (double t = t0; t <= len + 1e-9; t += step) {
      const double s = walked + (t - t0);
      if (s > lookahead) return -1.0;
      const double x = path[i].x + ux * t, y = path[i].y + uy * t;
      if (obstacles(x, y) >= c_narrow) { *point = {x, y}; return s; }
    }
    walked += len - t0;
  }
  return -1.0;
}

}  // namespace

SamplingController::SamplingController(ControllerParams p, std::shared_ptr<const MotionModel> model,
                                       std::shared_ptr<const Footprint> footprint)
    : p_(std::move(p)), model_(std::move(model)), footprint_(std::move(footprint)),
      social_(p_.social, footprint_) {
  if (!model_) throw std::invalid_argument("SamplingController: motion model is null");
  if (!footprint_) throw std::invalid_argument("SamplingController: footprint is null");
}

ControlResult SamplingController::compute(const Pose2D& pose, const Twist2D& vel,
                                          const std::vector<Pose2D>& path, const TrackedPeople& people,
                                          const ObstacleCostFn& obstacles, double speed_limit_ratio) {
  ControlResult result;
  Vec2 narrow_pt{0, 0}, tangent_r{1, 0};
  const double ratio = std::min(1.0, std::max(0.0, speed_limit_ratio));
  const double dt = p_.dt;
  const std::size_t n_steps = static_cast<std::size_t>(std::max(1.0, std::round(p_.horizon / dt)));

  // 速度上限: v_max * ratio。ただし加速度制約を破って急停止しないよう、現在速度から 1 周期で
  // 減速できる分は上限より上でも許す(ratio=0 でも段階的に減速して停止する)
  VelocityLimits lim = p_.limits;
  lim.v_max = std::max(p_.limits.v_max * ratio, std::min(p_.limits.v_max, vel.vx - p_.limits.acc_v * dt));
  lim.v_max = std::max(lim.v_max, lim.v_min);

  std::vector<Twist2D> cands;
  model_->sampleCommands(vel, lim, p_.samples, dt, cands);

  // door_yield: 発動なら vx がこの値を超える候補を捨てる(減速候補は必ず残る)
  bool yield_active = false;
  double yield_vx_max = 0.0;
  std::vector<Pose2D> shifted;   // door_shift != 0 のとき、保持中だけ経路の参照点を右へずらしたもの
  if (p_.door_yield && !path.empty()) {
    const double s_narrow = findNarrowPoint(pose, path, obstacles, p_.door_c_narrow, p_.door_lookahead, &narrow_pt, &tangent_r);
    result.door_narrow_s = s_narrow;
    if (door_cool_ > 0.0) door_cool_ = std::max(0.0, door_cool_ - dt);
    bool cond = false;
    if (s_narrow >= p_.door_d_commit && door_cool_ <= 0.0) {
      for (const Agent& a : people.agents) {
        if (a.confidence < p_.social.min_confidence) continue;
        if (std::hypot(a.pos.x - pose.x, a.pos.y - pose.y) > p_.door_range) continue;
        if ((a.pos.x - pose.x) * tangent_r.x + (a.pos.y - pose.y) * tangent_r.y < 0.0) continue;   // ロボットより後方へ抜けた人
        for (double t = 0.0; t <= p_.door_t_gate + 1e-9; t += dt)
          if (std::hypot(a.pos.x + a.vel.x * t - narrow_pt.x, a.pos.y + a.vel.y * t - narrow_pt.y) <= p_.door_r_gate) { cond = true; break; }
        if (cond) break;
      }
    }
    if (cond) {
      door_hold_ += dt;
      if (door_hold_ > p_.door_hold_max) { door_hold_ = 0.0; door_cool_ = p_.door_cooldown; cond = false; }
    } else {
      door_hold_ = 0.0;
    }
    if (cond) {
      const double v_cap = std::sqrt(2.0 * p_.limits.acc_v * std::max(0.0, s_narrow - p_.door_d_hold));
      yield_vx_max = std::max(v_cap, std::max(vel.vx - p_.limits.acc_v * dt, p_.limits.v_min)) + 1e-9;
      yield_active = true;
      result.door_yield_active = true;
      if (p_.door_shift != 0.0 && path.size() >= 2) {
        shifted = path;
        for (std::size_t i = 0; i < path.size(); ++i) {
          const std::size_t a = std::min(i, path.size() - 2);
          const double ux = path[a + 1].x - path[a].x, uy = path[a + 1].y - path[a].y;
          const double len = std::hypot(ux, uy);
          if (len < 1e-9) continue;
          shifted[i].x += p_.door_shift * uy / len;    // 進行方向の右側
          shifted[i].y -= p_.door_shift * ux / len;
        }
      }
    }
  }

  social_.prepare(people, {pose.x, pose.y}, dt, dt, n_steps);
  const bool use_social = p_.w_social != 0.0 && !social_.empty();

  const std::vector<Pose2D>& ref_path = shifted.empty() ? path : shifted;
  const bool has_path = !path.empty();
  const Pose2D goal = has_path ? ref_path.back() : pose;
  const double v_ref = std::max(p_.limits.v_max, 1e-6);

  // 譲るときの 1 周の案 H: 現在位置に最も近い経路区間の接線方向(1 周期に 1 回)。経路が 2 点未満、または長さ 0 の区間だけなら使わない
  const bool use_tie_heading = p_.tie_path_heading && !p_.tie_min_turn && !p_.tie_min_turn_stopped && path.size() >= 2;
  bool have_tangent = false;
  double tangent = 0.0;
  if (use_tie_heading) {
    double bd = std::numeric_limits<double>::max();
    for (std::size_t i = 0; i + 1 < path.size(); ++i) {
      const double sx = path[i + 1].x - path[i].x, sy = path[i + 1].y - path[i].y;
      if (std::hypot(sx, sy) < 1e-9) continue;
      const double d = distToSegment(pose.x, pose.y, path[i], path[i + 1]);
      if (d < bd) { bd = d; tangent = std::atan2(sy, sx); have_tangent = true; }
    }
  }
  double best_err = 0.0;

  std::vector<float> xs(n_steps), ys(n_steps), yaws(n_steps);
  std::vector<Vec2> query;
  double best = std::numeric_limits<double>::max();
  Twist2D best_cmd;
  bool found = false;
  bool best_contact = false;
  const bool do_diag = diag_ && use_social;
  if (do_diag) {
    result.diag.valid = true;
    std::fill(xs.begin(), xs.end(), static_cast<float>(pose.x));
    std::fill(ys.begin(), ys.end(), static_cast<float>(pose.y));
    std::fill(yaws.begin(), yaws.end(), static_cast<float>(pose.theta));
    result.diag.zero_free = social_.hasPredictedContact({xs.data(), ys.data(), yaws.data(), n_steps, 1}) ? 0 : 1;
  }

  // 候補を n_steps 先まで回す(障害物に触れたら collide。xs/ys/yaws は呼び出し側の作業領域)
  const auto rollout = [&](const Twist2D& c, Pose2D& q, bool& collide, double& obs_max, bool accel = false) {
    q = pose;
    collide = false;
    obs_max = 0.0;
    for (std::size_t k = 0; k < n_steps && !collide; ++k) {
      Twist2D t = c;
      if (accel) t.vx = std::min(lim.v_max, c.vx + p_.limits.acc_v * static_cast<double>(k + 1) * dt);
      q = model_->integrate(q, t, dt);
      xs[k] = static_cast<float>(q.x);
      ys[k] = static_cast<float>(q.y);
      yaws[k] = static_cast<float>(q.theta);
      if (obstacles) {
        query.clear();
        footprint_->costQueryPoints(q, query);
        for (const Vec2& pt : query) {
          const double o = obstacles(pt.x, pt.y);
          if (o >= 1.0) { collide = true; break; }
          obs_max = std::max(obs_max, o);
        }
      }
    }
  };

  // R2(stop_aware): 通過の候補(c.vx >= vel.vx)のどれかに予測接触がないときだけ、減速の候補を
  // 減速軌道(v_k = max(0, vel.vx - acc_v k dt)、ω = c.wz)で作り直し、予測接触があれば棄却する
  bool reject_decel = false;
  if (p_.stop_aware && use_social) {
    for (const Twist2D& c : cands) {
      if (c.vx < vel.vx) continue;
      Pose2D q;
      bool collide;
      double obs_max;
      rollout(c, q, collide, obs_max);
      if (collide) continue;
      if (!social_.hasPredictedContact({xs.data(), ys.data(), yaws.data(), n_steps, 1})) { reject_decel = true; break; }
    }
  }

  const auto max_vx_for_wz = [&](double wz) {
    double m = -std::numeric_limits<double>::max();
    for (const Twist2D& o : cands) if (o.wz == wz) m = std::max(m, o.vx);
    return m;
  };

  for (const Twist2D& c : cands) {
    if (yield_active && c.vx > yield_vx_max) continue;
    Pose2D q = pose;
    bool collide = false;
    double obs_max = 0.0;
    rollout(c, q, collide, obs_max);
    if (collide) continue;
    bool contact = false;
    if (do_diag) {
      const TrajectoryView dv{xs.data(), ys.data(), yaws.data(), n_steps, 1};
      contact = social_.hasPredictedContact(dv);
      ++result.diag.n_cand;
      if (!contact) ++result.diag.n_free;
    }

    if (reject_decel && c.vx < vel.vx) {
      Pose2D d = pose;
      std::vector<float> dx(n_steps), dy(n_steps), dyaw(n_steps);
      for (std::size_t k = 0; k < n_steps; ++k) {
        Twist2D t = c;
        t.vx = std::max(0.0, vel.vx - p_.limits.acc_v * static_cast<double>(k + 1) * dt);
        d = model_->integrate(d, t, dt);
        dx[k] = static_cast<float>(d.x);
        dy[k] = static_cast<float>(d.y);
        dyaw[k] = static_cast<float>(d.theta);
      }
      if (social_.hasPredictedContact({dx.data(), dy.data(), dyaw.data(), n_steps, 1})) continue;
    }

    const auto traj_cost = [&](const Pose2D& qe, double om) {
      double cost = p_.w_obstacle * om;
      if (has_path) {
        double sum = 0.0;
        for (std::size_t k = 0; k < n_steps; ++k) sum += distToPath(xs[k], ys[k], ref_path);
        cost += p_.w_path * sum / static_cast<double>(n_steps);
        cost += p_.w_goal * std::hypot(qe.x - goal.x, qe.y - goal.y);
      }
      cost += p_.w_speed * std::max(0.0, lim.v_max - c.vx) / v_ref;
      if (p_.w_turn != 0.0) cost += p_.w_turn * std::fabs(c.wz);
      if (p_.w_heading != 0.0 && has_path) {
        const double dx = goal.x - qe.x, dy = goal.y - qe.y;
        if (std::hypot(dx, dy) > 1e-6) {
          double e = std::atan2(dy, dx) - qe.theta;
          e = std::remainder(e, 2.0 * M_PI);
          cost += p_.w_heading * std::fabs(e);
        }
      }
      if (use_social) {
        const TrajectoryView view{xs.data(), ys.data(), yaws.data(), n_steps, 1};
        cost += p_.w_social * social_.score(view);
      }
      return cost;
    };
    double cost = traj_cost(q, obs_max);
    // accel_rollout: 6 m 以内(SocialCostParams.max_range)に追跡中の人がいない周期だけ、各 wz で vx 最大の候補を
    // 加速し続ける軌道でも評価し、低い方を採る(指令は c のまま)
    if (p_.accel_rollout && social_.empty() && c.vx >= max_vx_for_wz(c.wz)) {
      Pose2D qa;
      bool col_a;
      double om_a;
      rollout(c, qa, col_a, om_a, true);
      if (!col_a) cost = std::min(cost, traj_cost(qa, om_a));
    }
    double heading_err = 0.0;
    if (have_tangent) heading_err = std::fabs(std::remainder(tangent - q.theta, 2.0 * M_PI));
    const bool better = have_tangent
                            ? (cost < best - 1e-9 || (std::fabs(cost - best) <= 1e-9 && found && heading_err < best_err - 1e-9))
                        : (p_.tie_min_turn || (p_.tie_min_turn_stopped && ratio <= 0.0))
                            ? (cost < best - 1e-9 || (std::fabs(cost - best) <= 1e-9 && found && std::fabs(c.wz) < std::fabs(best_cmd.wz)))
                            : (cost < best);
    if (better) {
      best = cost;
      best_err = heading_err;
      best_cmd = c;
      best_contact = contact;
      found = true;
    }
  }

  if (!found) {
    result.all_colliding = true;
    return result;   // cmd は零指令、best_cost は 0
  }
  if (do_diag) result.diag.chosen_contact = best_contact ? 1 : 0;
  result.cmd = best_cmd;
  result.best_cost = best;
  return result;
}

}  // namespace social_nav
