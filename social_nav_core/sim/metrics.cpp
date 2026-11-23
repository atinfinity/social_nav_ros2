#include "sim/metrics.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace social_nav {
namespace sim {

MetricsRecorder::MetricsRecorder(std::shared_ptr<const Footprint> footprint, double intimate_dist,
                                 double personal_dist)
    : fp_(std::move(footprint)), intimate_(intimate_dist), personal_(personal_dist) {}

void MetricsRecorder::recordPhysics(const Pose2D& robot, const Twist2D& vel, const std::vector<Agent>& peds,
                                    bool safety_stop, double dt) {
  double min_d = 1e9;
  const Agent* nearest = nullptr;   // 診断: 中心間で最も近い人(M4 の方位別)
  const bool moving = std::hypot(vel.vx, vel.vy) >= 0.05;
  const int hist_n = static_cast<int>(std::lround(1.0 / dt));
  const double rvx = vel.vx * std::cos(robot.theta), rvy = vel.vx * std::sin(robot.theta);
  for (const Agent& a : peds) {
    const double d = std::hypot(robot.x - a.pos.x, robot.y - a.pos.y);
    double cr = 0, cp = 0;
    if (log_contacts_) {
      const double nx = d > 1e-9 ? (a.pos.x - robot.x) / d : 0.0, ny = d > 1e-9 ? (a.pos.y - robot.y) / d : 0.0;
      cr = rvx * nx + rvy * ny;
      cp = -(a.vel.x * nx + a.vel.y * ny);
      auto& h = close_hist_[a.id];
      h.emplace_back(cr * dt, cp * dt);
      while (static_cast<int>(h.size()) > hist_n) h.pop_front();
    }
    if (d < min_d) { min_d = d; nearest = &a; }
    const double c = fp_->clearance(robot, a.pos) - a.radius;   // M1, M2: 外形間
    m_.m2_min_clearance = std::min(m_.m2_min_clearance, c);
    bool& in = in_collision_[a.id];
    if (c < 0.0 && !in) {
      ++m_.m1_collisions;
      if (moving) { ++m_.m1_moving; ++m_.m1_moving_fast; }
      else if (stopped_dur_ < kM1StopSeconds) { ++m_.m1_moving; ++m_.m1_moving_recent; }
      else ++m_.m1_stopped;
      if (log_contacts_) {
        ContactEvent e;
        e.t = clock_ + dt;
        e.ped_id = a.id;
        e.robot_speed = std::hypot(vel.vx, vel.vy);
        e.cmd_speed = cmd_speed_;
        e.cmd_vx = cmd_vx_;
        e.speed_ratio = speed_ratio_;
        e.stopped_dur = stopped_dur_;
        e.hyst_dur = hyst_dur_;
        e.safety_stop = safety_stop;
        int ns = 0;
        for (const auto& hv : hist_) { e.max_speed_1s = std::max(e.max_speed_1s, hv.first); ns += hv.second ? 1 : 0; }
        e.safety_frac_1s = hist_.empty() ? 0.0 : static_cast<double>(ns) / static_cast<double>(hist_.size());
        int nc = 0, na = 0;
        for (const auto& nv : neg_hist_) { nc += nv.first ? 1 : 0; na += nv.second ? 1 : 0; }
        if (!neg_hist_.empty()) {
          e.neg_cmd_frac_1s = static_cast<double>(nc) / static_cast<double>(neg_hist_.size());
          e.neg_applied_frac_1s = static_cast<double>(na) / static_cast<double>(neg_hist_.size());
        }
        e.behavior = static_cast<int>(last_behavior_);
        e.behavior_age = clock_ - behavior_since_;
        e.close_robot = cr;
        e.close_ped = cp;
        for (const auto& hv : close_hist_[a.id]) { e.approach_robot_1s += hv.first; e.approach_ped_1s += hv.second; }
        e.ped_speed = std::hypot(a.vel.x, a.vel.y);
        double b = std::atan2(a.pos.y - robot.y, a.pos.x - robot.x) - robot.theta;
        e.bearing = std::atan2(std::sin(b), std::cos(b));
        auto le = last_contact_end_.find(a.id);
        e.prev_gap = le == last_contact_end_.end() ? -1.0 : clock_ - le->second;
        e.min_clear = c;
        if (auto ti = track_info_.find(a.id); ti != track_info_.end()) e.track = ti->second;
        for (auto it = diag_hist_.rbegin(); it != diag_hist_.rend(); ++it) {
          if (!e.diag_05s.valid && it->first <= e.t - 0.5 + 1e-9) e.diag_05s = it->second;
          if (it->first <= e.t - 1.0 + 1e-9) { e.diag_1s = it->second; break; }
        }
        e.cls = moving ? 0 : (stopped_dur_ < kM1StopSeconds ? 1 : 2);
        open_contact_[a.id] = static_cast<int>(contacts_.size());
        contacts_.push_back(e);
      }
    }
    if (log_contacts_ && c < 0.0) {
      ContactEvent& e = contacts_[static_cast<std::size_t>(open_contact_[a.id])];
      ++e.steps;
      e.min_clear = std::min(e.min_clear, c);
    } else if (log_contacts_ && in) {
      last_contact_end_[a.id] = clock_;
    }
    in = c < 0.0;
  }
  for (std::size_t i = 0; i < peds.size(); ++i)
    for (std::size_t j = i + 1; j < peds.size(); ++j) {
      const Agent &a = peds[i], &b = peds[j];
      const uint64_t lo = std::min(a.id, b.id), hi = std::max(a.id, b.id);
      bool& pin = pp_in_contact_[(lo << 32) | hi];
      const bool now = std::hypot(a.pos.x - b.pos.x, a.pos.y - b.pos.y) - a.radius - b.radius < 0.0;
      if (now && !pin) ++m_.ped_ped_contacts;
      pin = now;
    }
  stopped_dur_ = moving ? 0.0 : stopped_dur_ + dt;
  if (log_contacts_) {
    static constexpr double kExit[3] = {0.10, 0.15, 0.20};
    const double sp = std::hypot(vel.vx, vel.vy);
    for (int i = 0; i < 3; ++i) {
      if (hyst_stopped_[i] && sp >= kExit[i]) hyst_stopped_[i] = false;
      else if (!hyst_stopped_[i] && sp < 0.05) hyst_stopped_[i] = true;
      hyst_dur_[i] = hyst_stopped_[i] ? hyst_dur_[i] + dt : 0.0;
    }
    hist_.emplace_back(sp, safety_stop);
    while (static_cast<int>(hist_.size()) > hist_n) hist_.pop_front();
    neg_hist_.emplace_back(cmd_vx_ < 0.0, vel.vx < 0.0);
    while (static_cast<int>(neg_hist_.size()) > hist_n) neg_hist_.pop_front();
  }
  clock_ += dt;
  if (min_d < intimate_) m_.m3_intimate_time += dt;     // M3, M4: 中心間
  if (min_d < personal_) {
    m_.m4_personal_time += dt;
    // 診断: M4 の内訳(合計は m4_personal_time に一致)。状態は直近の recordBehavior、方位は最も近い人から見たロボットの向き
    m_.m4_by_behavior[static_cast<std::size_t>(last_behavior_)] += dt;
    M4Bearing bin = kM4NoDir;
    const double pv = std::hypot(nearest->vel.x, nearest->vel.y);
    if (pv >= kM4MinPedSpeed) {
      const double dx = robot.x - nearest->pos.x, dy = robot.y - nearest->pos.y;
      const double ang = std::acos(std::clamp((dx * nearest->vel.x + dy * nearest->vel.y) / (min_d * pv), -1.0, 1.0));
      bin = ang < kM4FrontRad ? kM4Front : (ang > kM4BackRad ? kM4Back : kM4Side);
    }
    m_.m4_by_bearing[bin] += dt;
  }
  if (std::hypot(vel.vx, vel.vy) < 0.05) m_.m8_stopped_time += dt;
  if (safety_stop && !safety_active_) ++m_.m11_safety_stops;
  safety_active_ = safety_stop;
  if (has_prev_) path_len_ += std::hypot(robot.x - prev_pose_.x, robot.y - prev_pose_.y);
  prev_pose_ = robot;
  has_prev_ = true;
}

void MetricsRecorder::recordBehavior(Behavior b, double dt) {
  if (has_behavior_ && b != last_behavior_) { ++m_.m9_transitions; behavior_since_ = clock_; }
  has_behavior_ = true;
  last_behavior_ = b;
  m_.m10_dwell[static_cast<std::size_t>(b)] += dt;
}

void MetricsRecorder::recordControlTime(double ms) {
  ctrl_ms_sum_ += ms;
  ++ctrl_n_;
  m_.m12_max_ms = std::max(m_.m12_max_ms, ms);
}

void MetricsRecorder::recordTracks(const std::vector<Agent>& truth, const TrackedPeople& tracks,
                                   double dt) {
  track_clock_ += dt;
  const double now = track_clock_;
  // M14: 各真値に最も近い確定 track(0.5 m 以内)の id が前回から変わった回数。重複の診断も同時に数える
  for (const Agent& g : truth) {
    const Agent* best = nullptr;
    double best_d = 0.5;
    int near_count = 0;
    for (const Agent& t : tracks.agents) {
      const double d = std::hypot(t.pos.x - g.pos.x, t.pos.y - g.pos.y);
      if (d <= 0.5) ++near_count;
      if (d < best_d) { best_d = d; best = &t; }
    }
    if (near_count >= 2) ++m_.dup_steps;
    if (!best) continue;
    auto it = last_track_id_.find(g.id);
    if (it == last_track_id_.end()) last_track_id_[g.id] = best->id;
    else if (it->second != best->id) {
      ++m_.m14_id_switches;
      it->second = best->id;
      truth_switch_time_[g.id] = now;
    }
  }
  track_matches_.clear();
  std::unordered_map<uint32_t, double> info_dist;   // 真値 id -> 採用した track の距離
  for (const Agent& g : truth) track_info_[g.id] = PedTrackInfo{};
  for (const Agent& t : tracks.agents) {
    track_first_seen_.emplace(t.id, now);  // 既にあれば何もしない
    const Agent* best = nullptr;
    double best_d = 0.5;
    for (const Agent& g : truth) {
      const double d = std::hypot(t.pos.x - g.pos.x, t.pos.y - g.pos.y);
      if (d < best_d) { best_d = d; best = &g; }
    }
    if (!best) continue;
    const double ex = t.vel.x - best->vel.x, ey = t.vel.y - best->vel.y;
    const double e2 = ex * ex + ey * ey;
    m_.m13_sumsq += e2;
    ++m_.m13_count;

    int cls = kM13Steady;
    const double age = now - track_first_seen_[t.id];
    auto sw = truth_switch_time_.find(best->id);
    auto pv = truth_prev_vel_.find(best->id);
    if (age < kM13TransientSeconds) {
      cls = kM13Transient;
    } else if (sw != truth_switch_time_.end() && now - sw->second < kM13SwitchSeconds) {
      cls = kM13Switch;
    } else if (pv != truth_prev_vel_.end() && dt > 0.0 &&
               std::hypot(best->vel.x - pv->second.x, best->vel.y - pv->second.y) / dt >
                   kM13AccelThreshold) {
      cls = kM13Accel;
    }
    m_.m13_class_sumsq[cls] += e2;
    ++m_.m13_class_count[cls];

    TrackMatch tm;
    tm.track_id = t.id;
    tm.truth_id = best->id;
    tm.truth_vel = best->vel;
    tm.dist = best_d;
    tm.info.has_track = true;
    tm.info.vel_err = std::sqrt(e2);
    tm.info.m13_class = cls;
    tm.info.track_age = age;
    track_matches_.push_back(tm);
    PedTrackInfo& cur = track_info_[best->id];
    if (!cur.has_track || best_d < info_dist[best->id]) { cur = tm.info; info_dist[best->id] = best_d; }
  }
  for (const Agent& g : truth) truth_prev_vel_[g.id] = g.vel;
}

RunMetrics MetricsRecorder::finish(bool reached, double reach_time, double straight_length) const {
  RunMetrics m = m_;
  m.reached = reached;
  m.m5_reach_time = reach_time;
  m.m7_path_ratio = straight_length > 1e-9 ? path_len_ / straight_length : 0.0;
  m.m12_mean_ms = ctrl_n_ > 0 ? ctrl_ms_sum_ / static_cast<double>(ctrl_n_) : 0.0;
  m.m13_rms = m.m13_count > 0 ? std::sqrt(m.m13_sumsq / static_cast<double>(m.m13_count)) : 0.0;
  return m;
}

}  // namespace sim
}  // namespace social_nav
