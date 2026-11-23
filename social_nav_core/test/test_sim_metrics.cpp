#include <cmath>
#include <memory>
#include <vector>

#include "mini_test.hpp"
#include "sim/metrics.hpp"
#include "social_nav_core/footprint.hpp"

using namespace social_nav;
using namespace social_nav::sim;

namespace {

constexpr double kDt = 0.02;

std::shared_ptr<const Footprint> circle() { return std::make_shared<CircleFootprint>(0.3); }

Agent staticPed(uint32_t id, double x, double y) {
  Agent a;
  a.id = id;
  a.pos = {x, y};
  a.radius = 0.3;
  return a;
}

// ロボットを x=0 で stop_s 秒静止させた後、1 m/s で x=10 まで直進させる
RunMetrics driveStraight(const std::vector<Agent>& peds, double stop_s) {
  MetricsRecorder rec(circle());
  Pose2D pose{0, 0, 0};
  for (int i = 0; i < static_cast<int>(stop_s / kDt + 0.5); ++i) rec.recordPhysics(pose, Twist2D{}, peds, false, kDt);
  for (int i = 0; i < 500; ++i) {
    pose.x += 1.0 * kDt;
    rec.recordPhysics(pose, Twist2D{1.0, 0, 0}, peds, false, kDt);
  }
  return rec.finish(true, 7.0, 10.0);
}

// 静止したロボット(原点)に、stop_s 秒後に歩行者が重なる
RunMetrics stoppedThenContact(double stop_s) {
  MetricsRecorder rec(circle());
  const std::vector<Agent> none;
  for (int i = 0; i < static_cast<int>(stop_s / kDt + 0.5); ++i) rec.recordPhysics({0, 0, 0}, Twist2D{}, none, false, kDt);
  rec.recordPhysics({0, 0, 0}, Twist2D{}, {staticPed(1, 0.5, 0.0)}, false, kDt);
  return rec.finish(false, 120.0, 10.0);
}

}  // namespace

TEST(sim_metrics_known_trajectory_m1_m3_m4_m8) {
  // ロボット中心 x(t) = 1.0 * t、人は (5, 0)。外形間 c = |5 - x| - 0.6、中心間 d = |5 - x|
  const RunMetrics m = driveStraight({staticPed(1, 5.0, 0.0)}, 2.0);
  EXPECT_TRUE(m.m1_collisions == 1);                 // x∈(4.4, 5.6) の 1 区間
  EXPECT_NEAR(m.m3_intimate_time, 0.9, 0.05);        // d<0.45: x∈(4.55, 5.45)
  EXPECT_NEAR(m.m4_personal_time, 2.4, 0.05);        // d<1.2: x∈(3.8, 6.2)
  EXPECT_NEAR(m.m8_stopped_time, 2.0, 0.05);         // 最初の 2 s 静止
  EXPECT_NEAR(m.m2_min_clearance, -0.6, 0.03);       // 中心が重なる付近
  EXPECT_NEAR(m.m7_path_ratio, 1.0, 0.01);
}

TEST(sim_metrics_collision_count_sums_over_pedestrians) {
  const RunMetrics m = driveStraight({staticPed(1, 3.0, 0.0), staticPed(2, 7.0, 0.1), staticPed(3, 5.0, 2.0)}, 0.0);
  EXPECT_TRUE(m.m1_collisions == 2);                 // 3 人目は通らない
  EXPECT_NEAR(m.m8_stopped_time, 0.0, 1e-9);
}

TEST(sim_metrics_m1_split_moving_and_stopped) {
  // 走行中に接触する人(x=5)と、静止中のロボットに最初から重なっている人
  const RunMetrics mv = driveStraight({staticPed(1, 5.0, 0.0)}, 0.0);
  EXPECT_TRUE(mv.m1_collisions == 1 && mv.m1_moving == 1 && mv.m1_stopped == 0);
  const RunMetrics st = stoppedThenContact(1.1);
  EXPECT_TRUE(st.m1_collisions == 1 && st.m1_moving == 0 && st.m1_stopped == 1);
}

TEST(sim_metrics_m1_boundary_stop_duration) {
  // 停止 0.9 s 後の接触 = M1a(走行中側)、1.1 s 後 = M1b
  const RunMetrics a = stoppedThenContact(0.9);
  EXPECT_TRUE(a.m1_collisions == 1 && a.m1_moving == 1 && a.m1_stopped == 0);
  const RunMetrics b = stoppedThenContact(1.1);
  EXPECT_TRUE(b.m1_collisions == 1 && b.m1_moving == 0 && b.m1_stopped == 1);
  // 走行 → 停止 → 0.9 s で接触でも M1a。途中で動くと停止継続はリセットされる
  MetricsRecorder rec(circle());
  const std::vector<Agent> none;
  for (int i = 0; i < 100; ++i) rec.recordPhysics({0, 0, 0}, Twist2D{}, none, false, kDt);        // 2 s 停止
  rec.recordPhysics({0, 0, 0}, Twist2D{0.5, 0, 0}, none, false, kDt);                              // 動く
  for (int i = 0; i < 45; ++i) rec.recordPhysics({0, 0, 0}, Twist2D{}, none, false, kDt);         // 0.9 s 停止
  rec.recordPhysics({0, 0, 0}, Twist2D{}, {staticPed(1, 0.5, 0.0)}, false, kDt);
  const RunMetrics r = rec.finish(false, 120.0, 10.0);
  EXPECT_TRUE(r.m1_moving == 1 && r.m1_stopped == 0);
}

TEST(sim_metrics_no_collision_when_passing_wide) {
  const RunMetrics m = driveStraight({staticPed(1, 5.0, 0.8)}, 0.0);   // 外形間 0.8-0.6 = 0.2 > 0
  EXPECT_TRUE(m.m1_collisions == 0);
  EXPECT_NEAR(m.m2_min_clearance, 0.2, 0.01);
  EXPECT_TRUE(m.m3_intimate_time == 0.0);            // d >= 0.8 > 0.45
  EXPECT_TRUE(m.m4_personal_time > 0.0);             // d<1.2 の区間がある
}

TEST(sim_metrics_m4_breakdown_sums_to_m4) {
  // 人 A(進行方向 +x、位置 (0,0))。ロボットを前(+x 側 0.5 m)・側(+y 側 0.5 m)・後(-x 側 0.5 m)・静止人の近くに置く
  Agent mover = staticPed(1, 0.0, 0.0);
  mover.vel = {1.0, 0.0};
  const Pose2D spots[] = {{0.5, 0, 0}, {0, 0.5, 0}, {-0.5, 0, 0}};
  MetricsRecorder rec(circle());
  rec.recordBehavior(Behavior::YIELD, 0.1);
  for (const Pose2D& p : spots) {
    for (int i = 0; i < 10; ++i) rec.recordPhysics(p, Twist2D{}, {mover}, false, kDt);   // 各 0.2 s
  }
  rec.recordBehavior(Behavior::WAIT, 0.1);
  for (int i = 0; i < 5; ++i) rec.recordPhysics({0.5, 0, 0}, Twist2D{}, {staticPed(2, 0.0, 0.0)}, false, kDt);   // 0.1 s、向きなし
  const RunMetrics m = rec.finish(false, 120.0, 10.0);
  EXPECT_NEAR(m.m4_personal_time, 0.7, 1e-9);
  EXPECT_NEAR(m.m4_by_behavior[static_cast<int>(Behavior::YIELD)], 0.6, 1e-9);
  EXPECT_NEAR(m.m4_by_behavior[static_cast<int>(Behavior::WAIT)], 0.1, 1e-9);
  EXPECT_NEAR(m.m4_by_bearing[kM4Front], 0.2, 1e-9);
  EXPECT_NEAR(m.m4_by_bearing[kM4Side], 0.2, 1e-9);
  EXPECT_NEAR(m.m4_by_bearing[kM4Back], 0.2, 1e-9);
  EXPECT_NEAR(m.m4_by_bearing[kM4NoDir], 0.1, 1e-9);
  double sb = 0, sg = 0;
  for (double v : m.m4_by_behavior) sb += v;
  for (double v : m.m4_by_bearing) sg += v;
  EXPECT_NEAR(sb, m.m4_personal_time, 1e-9);
  EXPECT_NEAR(sg, m.m4_personal_time, 1e-9);
}

TEST(sim_metrics_behavior_transitions_and_dwell) {
  MetricsRecorder rec(circle());
  const Behavior seq[] = {Behavior::PROCEED, Behavior::PROCEED, Behavior::YIELD, Behavior::YIELD,
                          Behavior::PROCEED, Behavior::FOLLOW};
  for (Behavior b : seq) rec.recordBehavior(b, 0.1);
  const RunMetrics m = rec.finish(false, 120.0, 10.0);
  EXPECT_TRUE(m.m9_transitions == 3);
  EXPECT_NEAR(m.m10_dwell[static_cast<int>(Behavior::YIELD)], 0.2, 1e-9);
  EXPECT_NEAR(m.m10_dwell[static_cast<int>(Behavior::PROCEED)], 0.3, 1e-9);
  EXPECT_TRUE(!m.reached);
}

TEST(sim_metrics_tracking_error_and_id_switch) {
  MetricsRecorder rec(circle());
  Agent truth = staticPed(1, 0.0, 0.0);
  truth.vel = {1.0, 0.0};
  TrackedPeople tp;
  Agent t = truth;
  t.id = 10;
  t.vel = {1.1, 0.0};
  tp.agents = {t};
  rec.recordTracks({truth}, tp);
  rec.recordTracks({truth}, tp);
  tp.agents[0].id = 11;   // 同一歩行者に別 id
  rec.recordTracks({truth}, tp);
  const RunMetrics m = rec.finish(true, 1.0, 1.0);
  EXPECT_NEAR(m.m13_rms, 0.1, 1e-9);
  EXPECT_TRUE(m.m14_id_switches == 1);
}

TEST(sim_metrics_m13_breakdown_four_classes_and_dup_steps) {
  // dt=0.125(2 進で厳密)。真値 1 人は原点。track 10 は原点、track 11 は (0.4,0) で、どちらも 0.5 m 以内
  const double dt = 0.125;
  MetricsRecorder rec(circle());
  Agent truth = staticPed(1, 0.0, 0.0);
  auto mk = [](uint32_t id, double x, double vx) {
    Agent a;
    a.id = id;
    a.pos = {x, 0.0};
    a.vel = {vx, 0.0};
    return a;
  };
  for (int step = 1; step <= 26; ++step) {
    if (step == 25) truth.vel = {0.2, 0.0};  // 加速度 1.6 m/s^2 > 0.5(この周期だけ)
    TrackedPeople tp;
    if (step <= 12) tp.agents.push_back(mk(10, 0.0, 0.1));
    tp.agents.push_back(mk(11, 0.4, 0.0));
    rec.recordTracks({truth}, tp, dt);
  }
  const RunMetrics m = rec.finish(true, 1.0, 1.0);
  EXPECT_TRUE(m.m13_class_count[kM13Transient] == 16);  // 1–8 周期 × 2 track
  EXPECT_TRUE(m.m13_class_count[kM13Switch] == 8);      // 13–20 周期(id 10 → 11)
  EXPECT_TRUE(m.m13_class_count[kM13Accel] == 1);       // 25 周期
  EXPECT_TRUE(m.m13_class_count[kM13Steady] == 13);  // 9–12 周期 × 2 track(8)+ 21–24, 26 周期(5)。総数 38−16−8−1
  long total = 0;
  double ss = 0;
  for (int c = 0; c < 4; ++c) { total += m.m13_class_count[c]; ss += m.m13_class_sumsq[c]; }
  EXPECT_TRUE(total == m.m13_count);
  EXPECT_NEAR(ss, m.m13_sumsq, 1e-9);
  EXPECT_TRUE(m.dup_steps == 12);  // 1–12 周期は 2 track が 0.5 m 以内
  EXPECT_TRUE(m.m14_id_switches == 1);
}

TEST(sim_metrics_safety_stop_counts_intervals) {
  MetricsRecorder rec(circle());
  const std::vector<Agent> none;
  const bool flags[] = {false, true, true, false, true, false};
  for (bool f : flags) rec.recordPhysics({0, 0, 0}, Twist2D{}, none, f, kDt);
  EXPECT_TRUE(rec.finish(true, 1.0, 1.0).m11_safety_stops == 2);
}

TEST(sim_metrics_contact_log_does_not_change_metrics) {
  // 停止 → 歩行者に接触 → 前進して通過。ログの有効・無効で指標が一致し、有効時は接触が記録される
  const std::vector<Agent> peds{staticPed(1, 0.5, 0.0), staticPed(2, 3.0, 0.2)};
  auto run = [&](bool log, std::size_t* n) {
    MetricsRecorder rec(circle());
    if (log) rec.enableContactLog();
    Pose2D pose{0, 0, 0};
    for (int i = 0; i < 100; ++i) rec.recordPhysics(pose, Twist2D{}, peds, i % 7 == 0, kDt);
    for (int i = 0; i < 300; ++i) {
      rec.setControlContext(1.0, 0.8);
      pose.x += 1.0 * kDt;
      rec.recordPhysics(pose, Twist2D{1.0, 0, 0}, peds, false, kDt);
    }
    if (n) *n = rec.contacts().size();
    return rec.finish(true, 8.0, 6.0);
  };
  std::size_t n_on = 0;
  const RunMetrics off = run(false, nullptr), on = run(true, &n_on);
  EXPECT_TRUE(n_on > 0);
  EXPECT_TRUE(on.m1_moving == off.m1_moving && on.m1_stopped == off.m1_stopped);
  EXPECT_TRUE(on.m1_moving_fast == off.m1_moving_fast && on.m1_moving_recent == off.m1_moving_recent);
  EXPECT_TRUE(on.m11_safety_stops == off.m11_safety_stops);
  EXPECT_NEAR(on.m2_min_clearance, off.m2_min_clearance, 0.0);
  EXPECT_NEAR(on.m3_intimate_time, off.m3_intimate_time, 0.0);
  EXPECT_NEAR(on.m4_personal_time, off.m4_personal_time, 0.0);
  EXPECT_NEAR(on.m8_stopped_time, off.m8_stopped_time, 0.0);
}

TEST(sim_metrics_contact_log_negative_vx_fractions) {
  // 静止歩行者(x=1.0)へ後退 → 前進接触。指令 vx<0 が 0.5 s、適用 vx<0 はその 0.25 s だけ(安全停止で 0 に)
  MetricsRecorder rec(circle());
  rec.enableContactLog();
  const std::vector<Agent> peds{staticPed(1, 1.0, 0.0)};
  const int n = static_cast<int>(std::lround(1.0 / kDt));
  Pose2D pose{0, 0, 0};
  for (int i = 0; i < n; ++i) {
    const bool first_half = i < n / 2;
    const bool applied_neg = i < n / 4;
    rec.setControlContext(0.5, 1.0, {}, first_half ? -0.5 : 0.5);
    rec.recordPhysics(pose, Twist2D{applied_neg ? -0.5 : 0.0, 0, 0}, peds, !applied_neg && first_half, kDt);
  }
  rec.setControlContext(0.7, 1.0, {}, 0.7);
  pose.x = 0.9;
  rec.recordPhysics(pose, Twist2D{0.7, 0, 0}, peds, false, kDt);
  EXPECT_TRUE(rec.contacts().size() == 1);
  const ContactEvent& e = rec.contacts()[0];
  EXPECT_NEAR(e.cmd_vx, 0.7, 1e-12);
  EXPECT_NEAR(e.neg_cmd_frac_1s, 0.5 * n / n, 0.03);
  EXPECT_NEAR(e.neg_applied_frac_1s, 0.25, 0.03);
}

TEST(sim_metrics_ped_track_info_and_matches_follow_m13_classification) {
  // M13 の分類テストと同じ並び(dt=0.125)。真値 1 人、track 10(原点、step 1–12)と 11(0.4,0)
  const double dt = 0.125;
  MetricsRecorder rec(circle());
  rec.enableContactLog();
  Agent truth = staticPed(1, 0.0, 0.0);
  auto mk = [](uint32_t id, double x, double vx) {
    Agent a;
    a.id = id; a.pos = {x, 0.0}; a.vel = {vx, 0.0};
    return a;
  };
  auto step = [&](int k) {
    if (k == 25) truth.vel = {0.2, 0.0};
    TrackedPeople tp;
    if (k <= 12) tp.agents.push_back(mk(10, 0.0, 0.1));
    tp.agents.push_back(mk(11, 0.4, 0.0));
    rec.recordTracks({truth}, tp, dt);
  };
  EXPECT_TRUE(!rec.pedTrackInfo(1).has_track && rec.pedTrackInfo(1).m13_class == -1);   // 呼ぶ前は既定値
  step(1);
  EXPECT_TRUE(rec.trackMatches().size() == 2);
  PedTrackInfo ti = rec.pedTrackInfo(1);   // 近い track 10 が採用される
  EXPECT_TRUE(ti.has_track && ti.m13_class == kM13Transient);
  EXPECT_NEAR(ti.vel_err, 0.1, 1e-12);
  EXPECT_NEAR(ti.track_age, 0.0, 1e-12);
  EXPECT_NEAR(rec.trackMatches()[0].truth_vel.x, 0.0, 1e-12);
  EXPECT_TRUE(rec.trackMatches()[0].truth_id == 1);
  for (int k = 2; k <= 10; ++k) step(k);
  EXPECT_TRUE(rec.pedTrackInfo(1).m13_class == kM13Steady);
  EXPECT_NEAR(rec.pedTrackInfo(1).track_age, 9 * dt, 1e-9);
  for (int k = 11; k <= 13; ++k) step(k);   // 13 で track 10 が消え、最近傍が 11 に変わる
  EXPECT_TRUE(rec.trackMatches().size() == 1 && rec.trackMatches()[0].track_id == 11);
  EXPECT_TRUE(rec.pedTrackInfo(1).m13_class == kM13Switch);
  for (int k = 14; k <= 25; ++k) step(k);
  EXPECT_TRUE(rec.pedTrackInfo(1).m13_class == kM13Accel);
  EXPECT_NEAR(rec.trackMatches()[0].truth_vel.x, 0.2, 1e-12);
  EXPECT_NEAR(rec.trackMatches()[0].info.vel_err, 0.2, 1e-12);

  // 対応する track がなくなったら has_track=false に戻り、接触イベントにも写る
  rec.recordTracks({truth}, TrackedPeople{}, dt);
  EXPECT_TRUE(!rec.pedTrackInfo(1).has_track && rec.trackMatches().empty());
  step(26);
  const std::vector<Agent> peds{staticPed(1, 0.5, 0.0), staticPed(2, -0.5, 0.0)};
  rec.recordPhysics({0, 0, 0}, Twist2D{}, peds, false, kDt);
  for (const ContactEvent& e : rec.contacts()) {
    if (e.ped_id == 1) EXPECT_TRUE(e.track.has_track);   // 真値 1 は原点にいないが対応は直近の recordTracks のもの
    else EXPECT_TRUE(!e.track.has_track && e.track.m13_class == -1 && std::isnan(e.track.vel_err));
  }
  EXPECT_TRUE(rec.contacts().size() == 2);
}

namespace {
// 人 2 人(id 1, 2。半径 0.3)を中心間距離 gap で置いて 1 ステップ流す
void stepPair(MetricsRecorder& rec, double gap) {
  rec.recordPhysics({0, 0, 0}, Twist2D{}, {staticPed(1, 5.0, 0.0), staticPed(2, 5.0 + gap, 0.0)}, false, kDt);
}
}  // namespace

TEST(sim_metrics_ped_ped_overlap_then_separate_counts_once) {
  MetricsRecorder rec(circle());
  for (double g : {2.0, 0.5, 0.4, 0.5, 2.0, 2.0}) stepPair(rec, g);
  EXPECT_TRUE(rec.finish(false, 1.0, 10.0).ped_ped_contacts == 1);
}

TEST(sim_metrics_ped_ped_staying_overlapped_counts_once) {
  MetricsRecorder rec(circle());
  for (int i = 0; i < 50; ++i) stepPair(rec, 0.4);
  EXPECT_TRUE(rec.finish(false, 1.0, 10.0).ped_ped_contacts == 1);
}

TEST(sim_metrics_ped_ped_recontact_counts_twice) {
  MetricsRecorder rec(circle());
  for (double g : {0.4, 0.4, 2.0, 2.0, 0.4, 2.0}) stepPair(rec, g);
  EXPECT_TRUE(rec.finish(false, 1.0, 10.0).ped_ped_contacts == 2);
}

TEST(sim_metrics_stopped_time_accumulates_only_while_stopped) {
  MetricsRecorder rec(circle());
  const std::vector<Agent> none;
  for (int i = 0; i < 100; ++i) rec.recordPhysics({0, 0, 0}, Twist2D{}, none, false, kDt);
  for (int i = 0; i < 100; ++i) rec.recordPhysics({0, 0, 0}, Twist2D{1.0, 0, 0}, none, false, kDt);
  for (int i = 0; i < 50; ++i) rec.recordPhysics({0, 0, 0}, Twist2D{}, none, false, kDt);
  EXPECT_NEAR(rec.finish(false, 1.0, 10.0).m8_stopped_time, 3.0, 0.03);
}

MINI_TEST_MAIN()
