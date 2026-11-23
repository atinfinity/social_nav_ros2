#include "social_nav_core/people_tracker.hpp"

#include "social_nav_core/assignment.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace social_nav {

namespace {
using M4 = std::array<double, 16>;
inline double& at(M4& m, int i, int j) { return m[i * 4 + j]; }
inline double at(const M4& m, int i, int j) { return m[i * 4 + j]; }

M4 mul(const M4& a, const M4& b) {
  M4 r{};
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j) {
      double s = 0;
      for (int k = 0; k < 4; ++k) s += at(a, i, k) * at(b, k, j);
      at(r, i, j) = s;
    }
  return r;
}
M4 transpose(const M4& a) {
  M4 r{};
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j) at(r, i, j) = at(a, j, i);
  return r;
}
constexpr double kMinVar = 1e-6;   // 観測共分散の下限(特異化防止)
}  // namespace

PeopleTracker::PeopleTracker(TrackerParams p) : p_(p) {}

void PeopleTracker::update(const std::vector<Detection>& dets, double stamp) {
  const double dt = has_stamp_ ? std::max(0.0, stamp - stamp_) : 0.0;
  has_stamp_ = true;
  stamp_ = stamp;

  // 予測(等速 + 加速度白色雑音)
  for (auto& t : tracks_) {
    t.x[0] += t.x[2] * dt;
    t.x[1] += t.x[3] * dt;
    M4 F{};
    for (int i = 0; i < 4; ++i) at(F, i, i) = 1.0;
    at(F, 0, 2) = dt;
    at(F, 1, 3) = dt;
    M4 Q{};
    const double q = p_.process_noise_acc * p_.process_noise_acc;
    for (int a = 0; a < 2; ++a) {
      at(Q, a, a) = q * dt * dt * dt * dt / 4.0;
      at(Q, a, a + 2) = at(Q, a + 2, a) = q * dt * dt * dt / 2.0;
      at(Q, a + 2, a + 2) = q * dt * dt;
    }
    M4 P = mul(mul(F, t.P), transpose(F));
    for (int i = 0; i < 16; ++i) P[i] += Q[i];
    t.P = P;
  }

  // 対応付け: ゲート内の組だけ許可。コスト = m2 + ln det S(P の大きい新規 track が検出を奪うのを防ぐ)。
  // max(track 数, 検出数) > 128 のときは計算時間を抑えるため距離順の貪欲に切り替える。
  constexpr std::size_t kMaxOptimalAssignment = 128;
  constexpr double kRecoveryGate = 5.0;   // 2 段目のゲート(σ)。TrackerParams には入れない
  const std::size_t nt = tracks_.size(), nd = dets.size();
  const double kInf = std::numeric_limits<double>::infinity();
  std::vector<std::vector<double>> allM2(nt, std::vector<double>(nd, kInf));   // det<=0 は無効(inf)
  std::vector<std::vector<double>> allLogDet(nt, std::vector<double>(nd, 0.0));
  const double gate2 = p_.gate_mahalanobis * p_.gate_mahalanobis;
  std::vector<std::vector<double>> cost(nt, std::vector<double>(nd, 0.0));
  std::vector<std::vector<bool>> forbidden(nt, std::vector<bool>(nd, true));
  struct Pair { double d2; std::size_t ti, di; };
  std::vector<Pair> pairs;
  for (std::size_t ti = 0; ti < nt; ++ti) {
    const auto& t = tracks_[ti];
    for (std::size_t di = 0; di < nd; ++di) {
      const double sxx = at(t.P, 0, 0) + std::max(dets[di].cov.xx, kMinVar);
      const double sxy = at(t.P, 0, 1) + dets[di].cov.xy;
      const double syy = at(t.P, 1, 1) + std::max(dets[di].cov.yy, kMinVar);
      const double det = sxx * syy - sxy * sxy;
      if (det <= 0) continue;
      const double ex = dets[di].pos.x - t.x[0], ey = dets[di].pos.y - t.x[1];
      const double m2 = (syy * ex * ex - 2 * sxy * ex * ey + sxx * ey * ey) / det;
      allM2[ti][di] = m2;
      allLogDet[ti][di] = std::log(det);
      if (m2 <= gate2) {
        pairs.push_back({m2, ti, di});
        cost[ti][di] = m2 + std::log(det);
        forbidden[ti][di] = false;
      }
    }
  }
  std::vector<std::pair<std::size_t, std::size_t>> matches;   // (track, 検出)
  if (std::max(nt, nd) > kMaxOptimalAssignment) {
    std::stable_sort(pairs.begin(), pairs.end(), [](const Pair& a, const Pair& b) { return a.d2 < b.d2; });
    std::vector<char> tu(nt, 0), du(nd, 0);
    for (const auto& pr : pairs) {
      if (tu[pr.ti] || du[pr.di]) continue;
      tu[pr.ti] = du[pr.di] = 1;
      matches.emplace_back(pr.ti, pr.di);
    }
  } else if (nt > 0 && nd > 0) {
    const auto asg = solveAssignment(cost, forbidden);
    for (std::size_t ti = 0; ti < nt; ++ti)
      if (asg[ti] >= 0) matches.emplace_back(ti, static_cast<std::size_t>(asg[ti]));
  }
  // 2 段目: 1 段目で残った確定 track と検出を広いゲートで再対応(新規 track の誕生=重複を防ぐ)。
  if (std::max(nt, nd) <= kMaxOptimalAssignment && nt > 0 && nd > 0) {
    std::vector<char> t1(nt, 0), d1(nd, 0);
    for (const auto& m : matches) { t1[m.first] = 1; d1[m.second] = 1; }
    std::vector<std::size_t> rt, rd;
    for (std::size_t ti = 0; ti < nt; ++ti)
      if (!t1[ti] && tracks_[ti].hits >= p_.confirm_hits) rt.push_back(ti);
    for (std::size_t di = 0; di < nd; ++di)
      if (!d1[di]) rd.push_back(di);
    if (!rt.empty() && !rd.empty()) {
      std::vector<std::vector<double>> c2(rt.size(), std::vector<double>(rd.size(), 0.0));
      std::vector<std::vector<bool>> f2(rt.size(), std::vector<bool>(rd.size(), true));
      for (std::size_t a = 0; a < rt.size(); ++a)
        for (std::size_t b = 0; b < rd.size(); ++b) {
          const double m2 = allM2[rt[a]][rd[b]];
          if (m2 <= kRecoveryGate * kRecoveryGate) {
            c2[a][b] = m2 + allLogDet[rt[a]][rd[b]];
            f2[a][b] = false;
          }
        }
      const auto asg2 = solveAssignment(c2, f2);
      for (std::size_t a = 0; a < rt.size(); ++a)
        if (asg2[a] >= 0) matches.emplace_back(rt[a], rd[static_cast<std::size_t>(asg2[a])]);
    }
  }
  std::vector<char> det_used(nd, 0);
  for (const auto& m : matches) {
    const auto pr = Pair{0, m.first, m.second};
    det_used[pr.di] = 1;

    auto& t = tracks_[pr.ti];
    const auto& d = dets[pr.di];
    const double sxx = at(t.P, 0, 0) + std::max(d.cov.xx, kMinVar);
    const double sxy = at(t.P, 0, 1) + d.cov.xy;
    const double syy = at(t.P, 1, 1) + std::max(d.cov.yy, kMinVar);
    const double det = sxx * syy - sxy * sxy;
    const double i00 = syy / det, i01 = -sxy / det, i11 = sxx / det;   // S^-1
    // K = P H^T S^-1(P の先頭 2 列 x S^-1)、4x2
    double K[4][2];
    for (int i = 0; i < 4; ++i) {
      K[i][0] = at(t.P, i, 0) * i00 + at(t.P, i, 1) * i01;
      K[i][1] = at(t.P, i, 0) * i01 + at(t.P, i, 1) * i11;
    }
    const double ex = d.pos.x - t.x[0], ey = d.pos.y - t.x[1];
    for (int i = 0; i < 4; ++i) t.x[i] += K[i][0] * ex + K[i][1] * ey;
    // P = (I - K H) P
    M4 P = t.P;
    for (int i = 0; i < 4; ++i)
      for (int j = 0; j < 4; ++j) at(P, i, j) = at(t.P, i, j) - K[i][0] * at(t.P, 0, j) - K[i][1] * at(t.P, 1, j);
    for (int i = 0; i < 4; ++i)
      for (int j = i + 1; j < 4; ++j) at(P, i, j) = at(P, j, i) = 0.5 * (at(P, i, j) + at(P, j, i));
    t.P = P;
    const double sp = std::hypot(t.x[2], t.x[3]);
    if (sp > p_.max_speed) {
      t.x[2] *= p_.max_speed / sp;
      t.x[3] *= p_.max_speed / sp;
    }
    t.hits += 1;
    t.last_update = stamp;
    t.radius = d.radius;
  }

  // 失った track の削除
  tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                               [&](const Track& t) { return stamp - t.last_update > p_.delete_after; }),
                tracks_.end());

  // 未対応の検出から新規 track
  for (std::size_t di = 0; di < dets.size(); ++di) {
    if (det_used[di]) continue;
    Track t;
    t.id = next_id_++;
    t.x = {dets[di].pos.x, dets[di].pos.y, 0.0, 0.0};
    t.P = M4{};
    at(t.P, 0, 0) = std::max(dets[di].cov.xx, kMinVar);
    at(t.P, 0, 1) = at(t.P, 1, 0) = dets[di].cov.xy;
    at(t.P, 1, 1) = std::max(dets[di].cov.yy, kMinVar);
    at(t.P, 2, 2) = at(t.P, 3, 3) = p_.init_vel_sigma * p_.init_vel_sigma;   // 初期速度の不確かさ
    t.hits = 1;
    t.last_update = stamp;
    t.radius = dets[di].radius;
    tracks_.push_back(t);
  }
}

TrackedPeople PeopleTracker::confirmed(const std::string& frame_id) const {
  TrackedPeople out;
  out.stamp = stamp_;
  out.frame_id = frame_id;
  for (const auto& t : tracks_) {
    if (t.hits < p_.confirm_hits) continue;
    Agent a;
    a.id = t.id;
    a.pos = {t.x[0], t.x[1]};
    a.vel = {t.x[2], t.x[3]};
    a.pos_cov = {at(t.P, 0, 0), at(t.P, 0, 1), at(t.P, 1, 1)};
    a.radius = t.radius;
    // 最後の更新からの経過に応じて信頼度を下げる(欠測中は 1 → 0)
    const double missed = std::max(0.0, stamp_ - t.last_update);
    a.confidence = p_.delete_after > 0 ? std::clamp(1.0 - missed / p_.delete_after, 0.0, 1.0) : 1.0;
    out.agents.push_back(a);
  }
  return out;
}

}  // namespace social_nav
