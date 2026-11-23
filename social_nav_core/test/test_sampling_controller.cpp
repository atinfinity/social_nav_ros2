#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include "mini_test.hpp"
#include "social_nav_core/sampling_controller.hpp"

using namespace social_nav;

namespace {

std::shared_ptr<const Footprint> circle() { return std::make_shared<CircleFootprint>(0.3); }
std::shared_ptr<const MotionModel> diff() { return std::make_shared<DiffDriveModel>(); }

std::vector<Pose2D> straightPath(double x0, double x1, double y = 0.0) {
  std::vector<Pose2D> p;
  for (double x = x0; x <= x1 + 1e-9; x += 0.1) p.push_back({x, y, 0.0});
  return p;
}

ObstacleCostFn none() { return [](double, double) { return 0.0; }; }

struct Run { Pose2D pose; Twist2D vel; double min_person_dist{1e9}; bool ever_all_colliding{false}; };

// 理想的な追従(指令がそのまま次の速度になる)で閉ループを回す
Run simulate(SamplingController& ctl, const MotionModel& model, Pose2D pose, const std::vector<Pose2D>& path,
             const TrackedPeople& people, int steps, double dt = 0.1) {
  Run r;
  r.pose = pose;
  for (int i = 0; i < steps; ++i) {
    const ControlResult res = ctl.compute(r.pose, r.vel, path, people, none());
    r.ever_all_colliding = r.ever_all_colliding || res.all_colliding;
    r.vel = res.cmd;
    r.pose = model.integrate(r.pose, res.cmd, dt);
    for (const Agent& a : people.agents)
      r.min_person_dist = std::min(r.min_person_dist, std::hypot(a.pos.x - r.pose.x, a.pos.y - r.pose.y));
  }
  return r;
}

TrackedPeople staticPerson(double x, double y) {
  TrackedPeople p;
  p.agents.push_back(Agent{1, {x, y}, {0, 0}, {}, 0.3, 1.0});
  return p;
}

// 差動二輪とは別の規則で動く偽モデル: 指令 vx で +y 方向へ進む。候補は 3 つだけ
class FakeModel final : public MotionModel {
 public:
  const char* name() const override { return "fake"; }
  Pose2D integrate(const Pose2D& pose, const Twist2D& cmd, double dt) const override {
    ++integrate_calls;
    Pose2D o = pose;
    o.y += cmd.vx * dt;
    return o;
  }
  void sampleCommands(const Twist2D&, const VelocityLimits&, const SampleSpec&, double,
                      std::vector<Twist2D>& out) const override {
    ++sample_calls;
    for (double v : {0.1, 0.2, 0.3}) { Twist2D t; t.vx = v; out.push_back(t); }
  }
  bool canRotateInPlace() const override { return false; }
  mutable int integrate_calls{0};
  mutable int sample_calls{0};
};

// 中心から x 方向に ±0.5 の 2 点を照会する偽フットプリント
class TwoPointFootprint final : public Footprint {
 public:
  double inscribedRadius() const override { return 0.3; }
  double circumscribedRadius() const override { return 0.6; }
  double clearance(const Pose2D& r, Vec2 p) const override { return std::hypot(p.x - r.x, p.y - r.y) - 0.5; }
  void costQueryPoints(const Pose2D& r, std::vector<Vec2>& out) const override {
    out.push_back({r.x - 0.5, r.y});
    out.push_back({r.x + 0.5, r.y});
  }
};

}  // namespace

TEST(controller_follows_path_to_goal) {
  SamplingController ctl(ControllerParams{}, diff(), circle());
  DiffDriveModel model;
  const auto path = straightPath(0.0, 5.0);
  // 経路から 0.5 m ずれ、向きも 0.3 rad ずれた位置から出発
  const Run r = simulate(ctl, model, {0.0, 0.5, 0.3}, path, TrackedPeople{}, 250);
  EXPECT_NEAR(r.pose.x, 5.0, 0.3);
  EXPECT_NEAR(r.pose.y, 0.0, 0.3);
  EXPECT_TRUE(!r.ever_all_colliding);
}

TEST(controller_stops_or_avoids_static_person_ahead) {
  SamplingController ctl(ControllerParams{}, diff(), circle());
  DiffDriveModel model;
  const auto path = straightPath(0.0, 6.0);
  const TrackedPeople people = staticPerson(3.0, 0.0);
  const Run r = simulate(ctl, model, {0.0, 0.0, 0.0}, path, people, 300);
  // 接触しない(ロボット半径 0.3 + 人半径 0.3)
  EXPECT_TRUE(r.min_person_dist > 0.6);
  // 人を通り越していなければ手前で停止、通り越していれば回避できたということ
  const bool stopped_before = r.pose.x < 3.0 && std::hypot(r.vel.vx, r.vel.wz) < 0.05;
  const bool passed = r.pose.x > 3.0;
  EXPECT_TRUE(stopped_before || passed);
}

TEST(controller_respects_acceleration_and_speed_limit_ratio) {
  const ControllerParams prm;
  SamplingController ctl(prm, diff(), circle());
  const auto path = straightPath(0.0, 5.0);
  const double eps = 1e-9;
  {
    // ratio 0.5: 上限 0.4。現在 0.3 から加速度制約内
    const Twist2D vel{0.3, 0.0, 0.1};
    const ControlResult r = ctl.compute({0, 0, 0}, vel, path, TrackedPeople{}, none(), 0.5);
    EXPECT_TRUE(r.cmd.vx <= prm.limits.v_max * 0.5 + eps);
    EXPECT_TRUE(std::fabs(r.cmd.vx - vel.vx) <= prm.limits.acc_v * prm.dt + eps);
    EXPECT_TRUE(std::fabs(r.cmd.wz - vel.wz) <= prm.limits.acc_w * prm.dt + eps);
    EXPECT_TRUE(r.cmd.vy == 0.0);
  }
  {
    // ratio 0: 加速度制約を守りつつ単調に減速して停止、再加速しない
    Twist2D vel{0.6, 0.0, 0.0};
    int stop_steps = -1;
    for (int i = 0; i < 20; ++i) {
      const ControlResult r = ctl.compute({0, 0, 0}, vel, path, TrackedPeople{}, none(), 0.0);
      EXPECT_TRUE(r.cmd.vx <= vel.vx + eps);
      EXPECT_TRUE(std::fabs(r.cmd.vx - vel.vx) <= prm.limits.acc_v * prm.dt + eps);
      vel = r.cmd;
      if (stop_steps < 0 && vel.vx <= eps) stop_steps = i + 1;
    }
    EXPECT_TRUE(stop_steps > 0 && stop_steps <= 7);
    EXPECT_NEAR(vel.vx, 0.0, eps);
  }
}

TEST(controller_over_limit_decelerates_at_max_rate_then_holds_limit) {
  const ControllerParams prm;
  SamplingController ctl(prm, diff(), circle());
  const auto path = straightPath(0.0, 5.0);
  const double eps = 1e-9;
  const double limit = prm.limits.v_max * 0.5;
  Twist2D vel{0.8, 0.0, 0.0};
  const int bound = static_cast<int>(std::ceil((0.8 - limit) / (prm.limits.acc_v * prm.dt) - 1e-9));
  bool reached = false;
  for (int i = 1; i <= bound + 20; ++i) {
    const ControlResult r = ctl.compute({0, 0, 0}, vel, path, TrackedPeople{}, none(), 0.5);
    if (vel.vx > limit + eps) EXPECT_TRUE(r.cmd.vx <= vel.vx - prm.limits.acc_v * prm.dt + eps);   // (a)
    vel = r.cmd;
    if (i == bound) { EXPECT_TRUE(vel.vx <= limit + eps); reached = true; }                      // (b)
    if (i >= bound) EXPECT_TRUE(vel.vx <= limit + eps);                                          // (c)
  }
  EXPECT_TRUE(reached);
}

TEST(controller_all_colliding_returns_stop) {
  SamplingController ctl(ControllerParams{}, diff(), circle());
  const ObstacleCostFn wall = [](double, double) { return 1.0; };
  const ControlResult r = ctl.compute({0, 0, 0}, {0.3, 0, 0}, straightPath(0, 5), TrackedPeople{}, wall);
  EXPECT_TRUE(r.all_colliding);
  EXPECT_NEAR(r.cmd.vx, 0.0, 1e-12);
  EXPECT_NEAR(r.cmd.vy, 0.0, 1e-12);
  EXPECT_NEAR(r.cmd.wz, 0.0, 1e-12);
}

TEST(controller_works_through_injected_fake_motion_model) {
  auto fake = std::make_shared<FakeModel>();
  const ControllerParams prm;
  SamplingController ctl(prm, fake, circle());
  // 経路は +y 方向。差動二輪(向き 0 → +x へ進む)ならこの経路には乗れない
  std::vector<Pose2D> path;
  for (double y = 0.0; y <= 1.0 + 1e-9; y += 0.1) path.push_back({0.0, y, 0.0});
  const ControlResult r = ctl.compute({0, 0, 0}, {}, path, TrackedPeople{}, none());
  EXPECT_TRUE(fake->sample_calls == 1);
  // 候補 3 本 × horizon/dt(30)ステップ、抽象越しにロールアウトしている
  EXPECT_TRUE(fake->integrate_calls == 3 * 30);
  // 偽モデルの規則では vx=0.3 が最もゴール(y=1)に近づく。返る指令は偽モデルの候補そのもの
  EXPECT_NEAR(r.cmd.vx, 0.3, 1e-12);
  EXPECT_TRUE(!r.all_colliding);
}

TEST(controller_uses_footprint_query_points_for_collisions) {
  // x >= 1.2 を衝突とする。円(中心 1 点)なら中心が 1.2 に届くまで安全、
  // 2 点フットプリントは前方点(中心 + 0.5)が先に触れる
  const ObstacleCostFn wall = [](double x, double) { return x >= 1.2 ? 1.0 : 0.0; };
  ControllerParams prm;
  prm.horizon = 1.0;
  const Twist2D vel{0.8, 0, 0};
  const auto path = straightPath(0.0, 5.0);
  SamplingController circ_ctl(prm, diff(), circle());
  SamplingController two_ctl(prm, diff(), std::make_shared<TwoPointFootprint>());
  const ControlResult rc = circ_ctl.compute({0.0, 0, 0}, vel, path, TrackedPeople{}, wall);
  const ControlResult r2 = two_ctl.compute({0.0, 0, 0}, vel, path, TrackedPeople{}, wall);
  EXPECT_TRUE(!rc.all_colliding);
  EXPECT_TRUE(rc.cmd.vx > 0.7);   // 1 s で 0.8 進んでも 1.2 に届かない
  EXPECT_TRUE(!r2.all_colliding);
  EXPECT_TRUE(r2.cmd.vx < rc.cmd.vx);   // 前方点が 1.2 に届かない遅い指令だけが残る
}

// R2: 通過の候補(vx >= vel.vx)に空きがあるとき、接触する減速の候補を棄却する
TEST(stop_aware_rejects_contacting_decel_candidates_when_passing_is_free) {
  TrackedPeople people;
  people.agents.push_back(Agent{1, {2.25, -1.25}, {-0.6, 0.5}, {}, 0.3, 1.0});   // 斜め前から来る(上の候補は通過、減速は接触)
  const Twist2D vel{0.8, 0.0, 0.0};
  const auto path = straightPath(0.0, 6.0);
  ControllerParams off;
  ControllerParams on;
  on.stop_aware = true;
  SamplingController c_off(off, diff(), circle());
  SamplingController c_on(on, diff(), circle());
  const ControlResult r_off = c_off.compute({0, 0, 0}, vel, path, people, none());
  const ControlResult r_on = c_on.compute({0, 0, 0}, vel, path, people, none());
  EXPECT_TRUE(r_off.cmd.vx < vel.vx - 1e-6);    // 現行は(接触する)減速の候補を選ぶ
  EXPECT_TRUE(r_on.cmd.vx >= vel.vx - 1e-6);    // 通過の候補が空きなので、減速の候補は棄却される
}

TEST(stop_aware_same_as_current_when_all_passing_candidates_contact) {
  // 目の前の静止した人: 通過の候補はすべて接触予測。on でも現行と同じ指令になる
  const Twist2D vel{0.8, 0.0, 0.0};
  const auto path = straightPath(0.0, 6.0);
  for (double px : {0.9, 1.2, 1.6}) {
    const TrackedPeople people = staticPerson(px, 0.0);
    ControllerParams on;
    on.stop_aware = true;
    SamplingController c_off(ControllerParams{}, diff(), circle());
    SamplingController c_on(on, diff(), circle());
    const ControlResult a = c_off.compute({0, 0, 0}, vel, path, people, none());
    const ControlResult b = c_on.compute({0, 0, 0}, vel, path, people, none());
    EXPECT_NEAR(a.cmd.vx, b.cmd.vx, 0.0);
    EXPECT_NEAR(a.cmd.wz, b.cmd.wz, 0.0);
    EXPECT_NEAR(a.best_cost, b.best_cost, 0.0);
  }
}

TEST(stop_aware_default_is_off) {
  EXPECT_TRUE(!ControllerParams{}.stop_aware);
}

TEST(stop_aware_has_no_effect_when_w_social_is_zero) {
  // baseline 構成(w_social = 0)では stop_aware = true でも現行と同じ指令になる
  TrackedPeople people;
  people.agents.push_back(Agent{1, {2.25, -1.25}, {-0.6, 0.5}, {}, 0.3, 1.0});
  const auto path = straightPath(0.0, 6.0);
  ControllerParams off;
  off.w_social = 0.0;
  ControllerParams on = off;
  on.stop_aware = true;
  SamplingController c_off(off, diff(), circle());
  SamplingController c_on(on, diff(), circle());
  const ControlResult a = c_off.compute({0, 0, 0}, {0.8, 0.0, 0.0}, path, people, none());
  const ControlResult b = c_on.compute({0, 0, 0}, {0.8, 0.0, 0.0}, path, people, none());
  EXPECT_NEAR(a.cmd.vx, b.cmd.vx, 0.0);
  EXPECT_NEAR(a.cmd.wz, b.cmd.wz, 0.0);
}

TEST(accel_rollout_default_is_off) {
  EXPECT_TRUE(!ControllerParams{}.accel_rollout);
}

TEST(accel_rollout_leaves_stalled_robot_through_costly_band) {
  // 停止中のロボットの前に、障害物コスト 0.5 の帯(x 1.8..2.2)。定速 3 s では帯の手前(中心が x < 1.8)に収まる低速しか選べない。
  // 加速継続なら帯を越えるので、on では上限(0.1)で前進する。off は従来と同じ(停止)
  const ObstacleCostFn band = [](double x, double) { return (x > 1.8 && x < 2.2) ? 0.5 : 0.0; };
  const auto path = straightPath(1.75, 6.0);
  ControllerParams off;
  ControllerParams on = off;
  on.accel_rollout = true;
  SamplingController c_off(off, diff(), circle());
  SamplingController c_on(on, diff(), circle());
  const ControlResult a = c_off.compute({1.75, 0, 0}, {0, 0, 0}, path, TrackedPeople{}, band);
  const ControlResult b = c_on.compute({1.75, 0, 0}, {0, 0, 0}, path, TrackedPeople{}, band);
  EXPECT_TRUE(a.cmd.vx < 0.02);
  EXPECT_TRUE(b.cmd.vx > 0.09);
}

TEST(accel_rollout_falls_back_to_constant_speed_when_accelerated_path_collides) {
  // 加速継続の先(x 3.0..3.2)にだけ衝突する壁がある: 定速の評価だけが残り、on と off は同じ
  const ObstacleCostFn wall = [](double x, double) {
    if (x > 3.0 && x < 3.2) return 1.0;
    return (x > 1.8 && x < 2.2) ? 0.5 : 0.0;
  };
  const auto path = straightPath(1.75, 6.0);
  ControllerParams off;
  ControllerParams on = off;
  on.accel_rollout = true;
  SamplingController c_off(off, diff(), circle());
  SamplingController c_on(on, diff(), circle());
  const ControlResult a = c_off.compute({1.75, 0, 0}, {0, 0, 0}, path, TrackedPeople{}, wall);
  const ControlResult b = c_on.compute({1.75, 0, 0}, {0, 0, 0}, path, TrackedPeople{}, wall);
  EXPECT_NEAR(a.cmd.vx, b.cmd.vx, 0.0);
  EXPECT_NEAR(a.cmd.wz, b.cmd.wz, 0.0);
  EXPECT_NEAR(a.best_cost, b.best_cost, 0.0);
}

TEST(accel_rollout_matches_off_without_obstacles) {
  const auto path = straightPath(0.0, 6.0);
  ControllerParams on;
  on.accel_rollout = true;
  SamplingController c_off(ControllerParams{}, diff(), circle());
  SamplingController c_on(on, diff(), circle());
  const ControlResult a = c_off.compute({0, 0, 0}, {0.3, 0, 0}, path, TrackedPeople{}, none());
  const ControlResult b = c_on.compute({0, 0, 0}, {0.3, 0, 0}, path, TrackedPeople{}, none());
  EXPECT_NEAR(a.cmd.vx, b.cmd.vx, 0.0);
  EXPECT_NEAR(a.cmd.wz, b.cmd.wz, 0.0);
}

TEST(accel_rollout_has_no_effect_when_a_person_is_within_range) {
  // 帯の手前で停止中(前のテストと同じ状況)でも、6 m 以内に人がいれば on と off は同じ指令・同じコスト
  const ObstacleCostFn band = [](double x, double) { return (x > 1.8 && x < 2.2) ? 0.5 : 0.0; };
  const auto path = straightPath(1.75, 6.0);
  ControllerParams on;
  on.accel_rollout = true;
  SamplingController c_off(ControllerParams{}, diff(), circle());
  SamplingController c_on(on, diff(), circle());
  const TrackedPeople people = staticPerson(1.75, 4.0);
  const ControlResult a = c_off.compute({1.75, 0, 0}, {0, 0, 0}, path, people, band);
  const ControlResult b = c_on.compute({1.75, 0, 0}, {0, 0, 0}, path, people, band);
  EXPECT_NEAR(a.cmd.vx, b.cmd.vx, 0.0);
  EXPECT_NEAR(a.cmd.wz, b.cmd.wz, 0.0);
  EXPECT_NEAR(a.best_cost, b.best_cost, 0.0);
}

// accel_rollout 境界: 「人なし」= 追跡中の人が全員 (距離 > max_range = 6 m) または (confidence < min_confidence = 0.3)。
// 距離 6 m ちょうどと confidence 0.3 ちょうどは「人あり」(trajectory_cost.cpp の prepare は `<` と `>` で除外する)
namespace {
struct BandRun { ControlResult off, on; };
BandRun runBand(const TrackedPeople& people) {
  const ObstacleCostFn band = [](double x, double) { return (x > 1.8 && x < 2.2) ? 0.5 : 0.0; };
  const auto path = straightPath(1.75, 6.0);
  ControllerParams on;
  on.accel_rollout = true;
  SamplingController c_off(ControllerParams{}, diff(), circle());
  SamplingController c_on(on, diff(), circle());
  return {c_off.compute({1.75, 0, 0}, {0, 0, 0}, path, people, band),
          c_on.compute({1.75, 0, 0}, {0, 0, 0}, path, people, band)};
}
}  // namespace

TEST(accel_rollout_person_at_exactly_max_range_counts_as_present) {
  // ロボット (1.75, 0)、人 (7.75, 0): 距離はちょうど 6.0。「人あり」なので on でも加速軌道を使わない
  const BandRun r = runBand(staticPerson(7.75, 0.0));
  EXPECT_NEAR(r.off.cmd.vx, r.on.cmd.vx, 0.0);
  EXPECT_NEAR(r.off.cmd.wz, r.on.cmd.wz, 0.0);
  EXPECT_NEAR(r.off.best_cost, r.on.best_cost, 0.0);
  EXPECT_TRUE(r.on.cmd.vx < 0.02);
}

TEST(accel_rollout_person_just_beyond_max_range_counts_as_absent) {
  // 距離 6.01 は「人なし」: 帯を越える加速軌道が使われ、上限(0.1)で前進する
  const BandRun r = runBand(staticPerson(7.76, 0.0));
  EXPECT_TRUE(r.off.cmd.vx < 0.02);
  EXPECT_TRUE(r.on.cmd.vx > 0.09);
}

TEST(accel_rollout_confidence_exactly_min_confidence_counts_as_present) {
  TrackedPeople p = staticPerson(1.75, 4.0);
  p.agents[0].confidence = SocialCostParams{}.min_confidence;   // 0.3 ちょうど
  const BandRun r = runBand(p);
  EXPECT_NEAR(r.off.cmd.vx, r.on.cmd.vx, 0.0);
  EXPECT_NEAR(r.off.cmd.wz, r.on.cmd.wz, 0.0);
  EXPECT_NEAR(r.off.best_cost, r.on.best_cost, 0.0);
  EXPECT_TRUE(r.on.cmd.vx < 0.02);
}

TEST(accel_rollout_confidence_just_below_min_confidence_counts_as_absent) {
  TrackedPeople p = staticPerson(1.75, 4.0);
  p.agents[0].confidence = std::nextafter(SocialCostParams{}.min_confidence, 0.0);   // 0.3 の直下
  const BandRun r = runBand(p);
  EXPECT_TRUE(r.off.cmd.vx < 0.02);
  EXPECT_TRUE(r.on.cmd.vx > 0.09);
}

// accel_rollout 1 ステップ目の先行: 加速軌道の k 歩目の速度は min(v_max, c.vx + acc_v (k+1) dt)、実際の適用速度は c.vx。
// horizon = dt(1 歩)、wz 候補 1 つ、vx 候補 2 つ(窓の両端)にして、障害物の問い合わせ点(円形は中心)の最大 x を取る。
// off は x0 + c.vx dt、on は x0 + min(v_max, c.vx + acc_v dt) dt なので、差 = (min(v_max, c.vx + acc_v dt) - c.vx) dt
namespace {
double maxQueriedX(bool accel, double vx0, double* applied_vx) {
  ControllerParams prm;
  prm.horizon = prm.dt;
  prm.samples.v_samples = 2;
  prm.samples.w_samples = 1;
  prm.accel_rollout = accel;
  double max_x = -1e9;
  const ObstacleCostFn rec = [&max_x](double x, double) { max_x = std::max(max_x, x); return 0.0; };
  SamplingController c(prm, diff(), circle());
  const ControlResult r = c.compute({0, 0, 0}, {vx0, 0, 0}, straightPath(0.0, 6.0), TrackedPeople{}, rec);
  if (applied_vx) *applied_vx = r.cmd.vx;
  return max_x;
}
}  // namespace

TEST(accel_rollout_first_step_lead_from_standstill) {
  // vx = 0: 窓 [0, 0.1]。適用 0.1(0.01 m)に対し、加速軌道は 0.2(0.02 m)。先行 = acc_v dt^2 = 0.01 m
  double va = -1, vb = -1;
  const double x_off = maxQueriedX(false, 0.0, &va);
  const double x_on = maxQueriedX(true, 0.0, &vb);
  EXPECT_NEAR(va, 0.1, 1e-12);
  EXPECT_NEAR(vb, 0.1, 1e-12);          // 指令は加速軌道の速度ではなく候補の vx
  EXPECT_NEAR(x_off, 0.1 * 0.1, 1e-12);
  EXPECT_NEAR(x_on, 0.2 * 0.1, 1e-12);
  EXPECT_NEAR(x_on - x_off, 0.01, 1e-12);
}

TEST(accel_rollout_first_step_lead_is_clipped_by_v_max) {
  // vx = 0.65: 窓 [0.55, 0.75]。加速軌道の 1 歩目は min(0.8, 0.85) = 0.8。先行 = (0.8 - 0.75) dt = 0.005 m
  const double x_off = maxQueriedX(false, 0.65, nullptr);
  const double x_on = maxQueriedX(true, 0.65, nullptr);
  EXPECT_NEAR(x_off, 0.75 * 0.1, 1e-12);
  EXPECT_NEAR(x_on, 0.8 * 0.1, 1e-12);
  EXPECT_NEAR(x_on - x_off, 0.005, 1e-12);
}

TEST(accel_rollout_first_step_lead_is_zero_at_v_max) {
  // vx = 0.8(上限): 候補の最大 0.8 は加速軌道でも 0.8。先行なし
  const double x_off = maxQueriedX(false, 0.8, nullptr);
  const double x_on = maxQueriedX(true, 0.8, nullptr);
  EXPECT_NEAR(x_off, 0.8 * 0.1, 1e-12);
  EXPECT_NEAR(x_on - x_off, 0.0, 1e-12);
}

// 譲るときの 1 周: 新しい項と tie_min_turn が全部 off(明示)のとき、既定と同じ指令・コスト(人がいる閉ループを数周期)
TEST(controller_83_flags_off_match_default) {
  ControllerParams off;
  off.tie_min_turn = false;
  off.w_turn = 0.0;
  off.w_heading = 0.0;
  SamplingController a(ControllerParams{}, diff(), circle());
  SamplingController b(off, diff(), circle());
  const auto path = straightPath(0.0, 6.0);
  const TrackedPeople people = staticPerson(3.0, 0.2);
  Pose2D pa{0, 0, 0}, pb{0, 0, 0};
  Twist2D va, vb;
  for (int i = 0; i < 20; ++i) {
    const ControlResult ra = a.compute(pa, va, path, people, none());
    const ControlResult rb = b.compute(pb, vb, path, people, none());
    EXPECT_NEAR(ra.cmd.vx, rb.cmd.vx, 0.0);
    EXPECT_NEAR(ra.cmd.wz, rb.cmd.wz, 0.0);
    EXPECT_NEAR(ra.best_cost, rb.best_cost, 0.0);
    va = ra.cmd; vb = rb.cmd;
    pa = DiffDriveModel().integrate(pa, va, 0.1);
    pb = DiffDriveModel().integrate(pb, vb, 0.1);
  }
}

// 譲るときの 1 周: vx = 0 の同点(speed_limit_ratio = 0、停止中)では、off は最小の wz、tie_min_turn は |wz| 最小を選ぶ
TEST(controller_83_tie_min_turn_picks_smallest_abs_wz) {
  ControllerParams on;
  on.tie_min_turn = true;
  SamplingController c_off(ControllerParams{}, diff(), circle());
  SamplingController c_on(on, diff(), circle());
  const auto path = straightPath(0.0, 6.0);
  const Twist2D vel{0.0, 0.0, -1.5};
  const ControlResult a = c_off.compute({0, 0, 0}, vel, path, TrackedPeople{}, none(), 0.0);
  const ControlResult b = c_on.compute({0, 0, 0}, vel, path, TrackedPeople{}, none(), 0.0);
  EXPECT_NEAR(a.cmd.vx, 0.0, 1e-9);
  EXPECT_NEAR(b.cmd.vx, 0.0, 1e-9);
  EXPECT_NEAR(a.cmd.wz, -1.5, 1e-9);
  EXPECT_TRUE(std::fabs(b.cmd.wz) < std::fabs(a.cmd.wz));
}

// 譲るときの 1 周: ratio = 0 を数周期続けると、tie_min_turn の wz は 0 に収束する(off は -1.5 のまま)
TEST(controller_83_tie_min_turn_converges_wz_to_zero) {
  ControllerParams on;
  on.tie_min_turn = true;
  SamplingController c_on(on, diff(), circle());
  const auto path = straightPath(0.0, 6.0);
  Twist2D vel{0.0, 0.0, -1.5};
  for (int i = 0; i < 8; ++i) vel = c_on.compute({0, 0, 0}, vel, path, TrackedPeople{}, none(), 0.0).cmd;
  EXPECT_NEAR(vel.vx, 0.0, 1e-9);
  EXPECT_NEAR(vel.wz, 0.0, 1e-9);
}

// 譲るときの 1 周の案 T2: tie_min_turn_stopped は ratio = 0 の同点で |wz| 最小を選ぶ(off は最小の wz)
TEST(controller_83_t2_stopped_tie_picks_smallest_abs_wz) {
  ControllerParams on;
  on.tie_min_turn_stopped = true;
  SamplingController c_off(ControllerParams{}, diff(), circle());
  SamplingController c_on(on, diff(), circle());
  const auto path = straightPath(0.0, 6.0);
  const Twist2D vel{0.0, 0.0, -1.5};
  const ControlResult a = c_off.compute({0, 0, 0}, vel, path, TrackedPeople{}, none(), 0.0);
  const ControlResult b = c_on.compute({0, 0, 0}, vel, path, TrackedPeople{}, none(), 0.0);
  EXPECT_NEAR(a.cmd.wz, -1.5, 1e-9);
  EXPECT_TRUE(std::fabs(b.cmd.wz) < std::fabs(a.cmd.wz));
}

// 譲るときの 1 周の案 T2: ratio > 0 では off と同じ指令・コスト(閉ループを数周期)
TEST(controller_83_t2_positive_ratio_matches_off) {
  ControllerParams on;
  on.tie_min_turn_stopped = true;
  SamplingController c_off(ControllerParams{}, diff(), circle());
  SamplingController c_on(on, diff(), circle());
  const auto path = straightPath(0.0, 6.0);
  Twist2D va{0.0, 0.0, -1.5}, vb = va;
  Pose2D pa{0, 0, 0}, pb{0, 0, 0};
  for (int i = 0; i < 10; ++i) {
    const double r = (i % 2) ? 0.5 : 1.0;
    const ControlResult ra = c_off.compute(pa, va, path, TrackedPeople{}, none(), r);
    const ControlResult rb = c_on.compute(pb, vb, path, TrackedPeople{}, none(), r);
    EXPECT_NEAR(ra.cmd.vx, rb.cmd.vx, 0.0);
    EXPECT_NEAR(ra.cmd.wz, rb.cmd.wz, 0.0);
    EXPECT_NEAR(ra.best_cost, rb.best_cost, 0.0);
    va = ra.cmd; vb = rb.cmd;
    pa = DiffDriveModel().integrate(pa, va, 0.1);
    pb = DiffDriveModel().integrate(pb, vb, 0.1);
  }
}

// 譲るときの 1 周の案 T2: off(明示)は現行と同じ(ratio = 0 でも最小の wz のまま)
TEST(controller_83_t2_off_matches_default) {
  ControllerParams off;
  off.tie_min_turn_stopped = false;
  SamplingController c_def(ControllerParams{}, diff(), circle());
  SamplingController c_off(off, diff(), circle());
  const auto path = straightPath(0.0, 6.0);
  const Twist2D vel{0.0, 0.0, -1.5};
  const ControlResult a = c_def.compute({0, 0, 0}, vel, path, TrackedPeople{}, none(), 0.0);
  const ControlResult b = c_off.compute({0, 0, 0}, vel, path, TrackedPeople{}, none(), 0.0);
  EXPECT_NEAR(a.cmd.wz, b.cmd.wz, 0.0);
  EXPECT_NEAR(a.best_cost, b.best_cost, 0.0);
}

// 譲るときの 1 周の案 H: off(明示)は現行と同じ(ratio = 0 でも最小の wz のまま)
TEST(controller_83_h_off_matches_default) {
  ControllerParams off;
  off.tie_path_heading = false;
  SamplingController c_def(ControllerParams{}, diff(), circle());
  SamplingController c_off(off, diff(), circle());
  const auto path = straightPath(0.0, 6.0);
  const Twist2D vel{0.0, 0.0, -1.5};
  const ControlResult a = c_def.compute({0, 0, 0}, vel, path, TrackedPeople{}, none(), 0.0);
  const ControlResult b = c_off.compute({0, 0, 0}, vel, path, TrackedPeople{}, none(), 0.0);
  EXPECT_NEAR(a.cmd.wz, b.cmd.wz, 0.0);
  EXPECT_NEAR(a.best_cost, b.best_cost, 0.0);
}

// 譲るときの 1 周の案 H: vx = 0 の同点では、終点の向きが経路の接線に近い候補を選ぶ(経路の向きで wz の符号が変わる)
TEST(controller_83_h_tie_picks_smallest_heading_error) {
  ControllerParams on;
  on.tie_path_heading = true;
  SamplingController c_on(on, diff(), circle());
  SamplingController c_off(ControllerParams{}, diff(), circle());
  const Twist2D vel{0.0, 0.0, 0.0};
  // 経路が +x: 向き 0 に最も近い wz = 0
  const ControlResult a = c_on.compute({0, 0, 0}, vel, straightPath(0.0, 6.0), TrackedPeople{}, none(), 0.0);
  EXPECT_NEAR(a.cmd.vx, 0.0, 1e-9);
  EXPECT_NEAR(a.cmd.wz, 0.0, 1e-9);
  // 経路が +y / -y: 回る向きが逆になる
  std::vector<Pose2D> up, down;
  for (double y = 0.0; y <= 6.0 + 1e-9; y += 0.1) { up.push_back({0, y, 0}); down.push_back({0, -y, 0}); }
  const ControlResult u = c_on.compute({0, 0, 0}, vel, up, TrackedPeople{}, none(), 0.0);
  const ControlResult d = c_on.compute({0, 0, 0}, vel, down, TrackedPeople{}, none(), 0.0);
  EXPECT_NEAR(u.cmd.vx, 0.0, 1e-9);
  EXPECT_TRUE(u.cmd.wz > 0.0);
  EXPECT_TRUE(d.cmd.wz < 0.0);
  EXPECT_NEAR(u.cmd.wz, -d.cmd.wz, 1e-9);
  // off は経路の向きに関係なく最小の wz
  const ControlResult o = c_off.compute({0, 0, 0}, vel, up, TrackedPeople{}, none(), 0.0);
  EXPECT_TRUE(o.cmd.wz < 0.0);
}

// 譲るときの 1 周の案 H: 経路なし / 1 点のとき、現行と同じ
TEST(controller_83_h_no_path_matches_default) {
  ControllerParams on;
  on.tie_path_heading = true;
  SamplingController c_on(on, diff(), circle());
  SamplingController c_def(ControllerParams{}, diff(), circle());
  const Twist2D vel{0.0, 0.0, -1.5};
  for (const std::vector<Pose2D>& path : {std::vector<Pose2D>{}, std::vector<Pose2D>{{3.0, 3.0, 0.0}}}) {
    const ControlResult a = c_on.compute({0, 0, 0}, vel, path, TrackedPeople{}, none(), 0.0);
    const ControlResult b = c_def.compute({0, 0, 0}, vel, path, TrackedPeople{}, none(), 0.0);
    EXPECT_NEAR(a.cmd.vx, b.cmd.vx, 0.0);
    EXPECT_NEAR(a.cmd.wz, b.cmd.wz, 0.0);
    EXPECT_NEAR(a.best_cost, b.best_cost, 0.0);
  }
}

// 譲るときの 1 周の案 H: コストが違えば e は見ない(ratio = 1 の通常走行は現行と同じ指令)
TEST(controller_83_h_cost_difference_ignores_heading) {
  ControllerParams on;
  on.tie_path_heading = true;
  SamplingController c_on(on, diff(), circle());
  SamplingController c_def(ControllerParams{}, diff(), circle());
  const auto path = straightPath(0.0, 6.0);
  Pose2D pa{0, 0, 0}, pb{0, 0, 0};
  Twist2D va, vb;
  for (int i = 0; i < 20; ++i) {
    const ControlResult ra = c_on.compute(pa, va, path, staticPerson(3.0, 0.2), none());
    const ControlResult rb = c_def.compute(pb, vb, path, staticPerson(3.0, 0.2), none());
    EXPECT_NEAR(ra.cmd.vx, rb.cmd.vx, 0.0);
    EXPECT_NEAR(ra.cmd.wz, rb.cmd.wz, 0.0);
    EXPECT_NEAR(ra.best_cost, rb.best_cost, 0.0);
    va = ra.cmd; vb = rb.cmd;
    pa = DiffDriveModel().integrate(pa, va, 0.1);
    pb = DiffDriveModel().integrate(pb, vb, 0.1);
  }
}

MINI_TEST_MAIN()
