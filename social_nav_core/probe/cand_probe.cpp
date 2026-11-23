// cand_probe: sim の --trace(F 行)から、ある制御周期の入力(pose、人、速度)を復元し、
// SamplingController と同じ候補について費用の内訳と最接近すきま g を出す(接触直前の止まり方の診断。ROS 非依存)。
// 使い方: cand_probe TRACE T [T ...]   T = 周期の開始時刻 [s](0.1 の倍数)。接触の 3 s 前と 1 s 前を 1 つずつ渡す。
// 近似: 速度は 0.1 s の差分(ロボット、人とも)。戦術層は trace から cycle 1 以降を再生して speed_ratio を得る。
// 照合: 自前の argmin、SamplingController::compute の cmd、trace の次の周期の動き(v, ω)を並べる。
// 置き場所は sim/ の外(GLOB に入れない)。ctest には入れず tools/run_cand_probe.sh で実行する。
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "sim/social_force.hpp"
#include "social_nav_core/footprint.hpp"
#include "social_nav_core/motion_model.hpp"
#include "social_nav_core/sampling_controller.hpp"
#include "social_nav_core/tactical_selector.hpp"

using namespace social_nav;
using namespace social_nav::sim;

namespace {

// sim/main.cpp と同じ値
constexpr double kControlDt = 0.1;
constexpr double kPhysicsDt = 0.02;
constexpr double kPathWindow = 4.0;
constexpr double kPathSpacing = 0.5;
constexpr int kSub = 5;   // 物理ステップ / 制御周期

struct Frame { Pose2D pose; std::vector<Vec2> peds; std::string behavior; };

struct Trace {
  std::string scenario, controller;
  int seed{0};
  std::vector<Segment> walls;
  Vec2 start, goal;
  double robot_r{0.3}, ped_r{0.3};
  std::vector<Frame> frames;   // frames[i] = 物理ステップ i+1(時刻 (i+1)*0.02)
};

bool load(const char* path, Trace& t) {
  std::ifstream f(path);
  if (!f) return false;
  std::string line;
  while (std::getline(f, line)) {
    std::istringstream is(line);
    std::string tag;
    is >> tag;
    if (tag == "H") is >> t.scenario >> t.seed >> t.controller;
    else if (tag == "W") { Segment s; is >> s.a.x >> s.a.y >> s.b.x >> s.b.y; t.walls.push_back(s); }
    else if (tag == "S") is >> t.start.x >> t.start.y;
    else if (tag == "G") is >> t.goal.x >> t.goal.y;
    else if (tag == "R") is >> t.robot_r >> t.ped_r;
    else if (tag == "F") {
      Frame fr;
      double time; int n;
      is >> time >> fr.pose.x >> fr.pose.y >> fr.pose.theta >> fr.behavior >> n;
      for (int i = 0; i < n; ++i) { Vec2 p; is >> p.x >> p.y; fr.peds.push_back(p); }
      t.frames.push_back(fr);
    }
  }
  return !t.frames.empty();
}

// 物理ステップ番号 s(1 始まり)の状態。s = 0 は開始時(S、停止、人は s=1 で代用)
const Frame& at(const Trace& t, long s) {
  return t.frames[static_cast<std::size_t>(std::max(1L, std::min<long>(s, static_cast<long>(t.frames.size()))) - 1)];
}

std::vector<Pose2D> pathWindow(const Vec2& start, Vec2 goal, const Pose2D& robot) {
  const double dx = goal.x - start.x, dy = goal.y - start.y;
  const double len = std::hypot(dx, dy);
  const double ux = dx / len, uy = dy / len;
  const double th = std::atan2(dy, dx);
  const double s0 = std::min(len, std::max(0.0, (robot.x - start.x) * ux + (robot.y - start.y) * uy));
  std::vector<Pose2D> w;
  for (double s = s0; s < std::min(len, s0 + kPathWindow); s += kPathSpacing)
    w.push_back({start.x + ux * s, start.y + uy * s, th});
  w.push_back({start.x + ux * std::min(len, s0 + kPathWindow), start.y + uy * std::min(len, s0 + kPathWindow), th});
  return w;
}

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
  for (std::size_t i = 0; i + 1 < path.size(); ++i) best = std::min(best, distToSegment(px, py, path[i], path[i + 1]));
  return best;
}

// 周期 k の開始時のロボット速度(0.1 s の差分。前進方向への射影と角度差)
Twist2D robotVel(const Trace& t, long k) {
  if (k < 2) return {};
  const Pose2D& a = at(t, kSub * (k - 1)).pose;
  const Pose2D& b = at(t, kSub * k).pose;
  const double mid = 0.5 * (a.theta + b.theta);
  Twist2D v;
  v.vx = ((b.x - a.x) * std::cos(mid) + (b.y - a.y) * std::sin(mid)) / kControlDt;
  v.wz = (b.theta - a.theta) / kControlDt;
  return v;
}

TrackedPeople peopleAt(const Trace& t, long k) {
  TrackedPeople ppl;
  ppl.frame_id = "map";
  ppl.stamp = static_cast<double>(k) * kControlDt;
  const Frame& cur = at(t, kSub * k);
  const Frame& ref = k >= 2 ? at(t, kSub * (k - 1)) : at(t, kSub * (k + 1));
  const double sgn = k >= 2 ? 1.0 : -1.0;
  for (std::size_t i = 0; i < cur.peds.size(); ++i) {
    Agent a;
    a.id = static_cast<uint32_t>(i + 1);
    a.pos = cur.peds[i];
    a.vel = {sgn * (cur.peds[i].x - ref.peds[i].x) / kControlDt, sgn * (cur.peds[i].y - ref.peds[i].y) / kControlDt};
    a.pos_cov = {0.01, 0.0, 0.01};
    a.radius = t.ped_r;
    a.confidence = 1.0;
    ppl.agents.push_back(a);
  }
  return ppl;
}

struct Row {
  Twist2D c; bool collide{false};
  double path{0}, goal{0}, speed{0}, prox{0}, coll{0}, obs{0}, total{0}, g{1e9};
};

void probeAt(const Trace& t, long k, double w_speed_default_unused) {
  (void)w_speed_default_unused;
  const double now = static_cast<double>(k) * kControlDt;
  FootprintSpec fspec;
  fspec.radius = t.robot_r;
  std::shared_ptr<const Footprint> fp = makeFootprint(fspec);
  std::shared_ptr<const MotionModel> model = makeMotionModel("diff_drive");
  ControllerParams cp;
  cp.w_speed = 1.0;   // sim と同じ
  SamplingController controller(cp, model, fp);
  SocialTrajectoryCost social(cp.social, fp);

  // 戦術層を cycle 1 から k まで再生
  TacticalSelector tactical{TacticalParams{}};
  TacticalDecision dec;
  for (long j = 1; j <= k; ++j) {
    const Pose2D pj = at(t, kSub * j).pose;
    const std::vector<Pose2D> pathj = pathWindow(t.start, t.goal, pj);
    const TrackedPeople pplj = peopleAt(t, j);
    TacticalInput in;
    in.now = static_cast<double>(j) * kControlDt;
    in.robot = pj;
    in.path = &pathj;
    in.people = &pplj;
    dec = tactical.step(in);
  }

  const Pose2D pose = at(t, kSub * k).pose;
  const Twist2D vel = robotVel(t, k);
  const TrackedPeople people = peopleAt(t, k);
  const std::vector<Pose2D> path = pathWindow(t.start, t.goal, pose);
  const double kRobotRadius = t.robot_r;
  const std::vector<Segment>& walls = t.walls;
  const ObstacleCostFn obstacles = [&walls, kRobotRadius](double x, double y) {
    const double d = distanceToWalls({x, y}, walls);
    if (d < kRobotRadius + 0.05) return 1.0;
    return 0.9 * std::exp(-(d - kRobotRadius - 0.05) / 0.25);
  };

  const double dt = cp.dt;
  const std::size_t n_steps = static_cast<std::size_t>(std::max(1.0, std::round(cp.horizon / dt)));
  VelocityLimits lim = cp.limits;
  lim.v_max = std::max(cp.limits.v_max * dec.speed_ratio, std::min(cp.limits.v_max, vel.vx - cp.limits.acc_v * dt));
  lim.v_max = std::max(lim.v_max, lim.v_min);
  std::vector<Twist2D> cands;
  model->sampleCommands(vel, lim, cp.samples, dt, cands);
  social.prepare(people, {pose.x, pose.y}, dt, dt, n_steps);

  // 予測位置(prepare と同じ: 等速、t_offset = dt)。max_range 内の人だけ g に使う
  std::vector<const Agent*> near;
  for (const Agent& a : people.agents)
    if (std::hypot(a.pos.x - pose.x, a.pos.y - pose.y) <= cp.social.max_range) near.push_back(&a);

  const Pose2D goal = path.back();
  const double v_ref = std::max(cp.limits.v_max, 1e-6);
  std::vector<float> xs(n_steps), ys(n_steps), yaws(n_steps);
  std::vector<Row> rows;
  std::vector<Vec2> query;
  for (const Twist2D& c : cands) {
    Row r;
    r.c = c;
    Pose2D q = pose;
    for (std::size_t i = 0; i < n_steps; ++i) {
      q = model->integrate(q, c, dt);
      xs[i] = static_cast<float>(q.x); ys[i] = static_cast<float>(q.y); yaws[i] = static_cast<float>(q.theta);
      query.clear();
      fp->costQueryPoints(q, query);
      for (const Vec2& pt : query) {
        const double o = obstacles(pt.x, pt.y);
        if (o >= 1.0) r.collide = true;
        r.obs = std::max(r.obs, o);
      }
      for (const Agent* a : near) {
        const Agent pa = predictConstantVelocity(*a, dt + static_cast<double>(i) * dt);
        const Pose2D pq{xs[i], ys[i], static_cast<double>(yaws[i])};
        r.g = std::min(r.g, fp->clearance(pq, pa.pos) - a->radius);
      }
      if (r.collide) break;
    }
    if (r.collide) { rows.push_back(r); continue; }
    double sum = 0.0;
    for (std::size_t i = 0; i < n_steps; ++i) sum += distToPath(xs[i], ys[i], path);
    r.path = cp.w_path * sum / static_cast<double>(n_steps);
    r.goal = cp.w_goal * std::hypot(q.x - goal.x, q.y - goal.y);
    r.speed = cp.w_speed * std::max(0.0, lim.v_max - c.vx) / v_ref;
    double pp = 0.0, cc = 0.0;
    social.scoreParts({xs.data(), ys.data(), yaws.data(), n_steps, 1}, pp, cc);
    r.prox = cp.w_social * pp;
    r.coll = cp.w_social * cc;
    r.obs = cp.w_obstacle * r.obs;
    r.total = r.obs + r.path + r.goal + r.speed + r.prox + r.coll;
    rows.push_back(r);
  }
  std::size_t best = rows.size();
  for (std::size_t i = 0; i < rows.size(); ++i)
    if (!rows[i].collide && (best == rows.size() || rows[i].total < rows[best].total)) best = i;

  const ControlResult res = controller.compute(pose, vel, path, people, obstacles, dec.speed_ratio);

  // trace の次の周期の動き(周期 k の 5 物理ステップ後の pose との差分 / 0.1 s)
  Twist2D next;
  {
    const Pose2D& a = pose;
    const Pose2D& b = at(t, kSub * (k + 1)).pose;
    const double mid = 0.5 * (a.theta + b.theta);
    next.vx = ((b.x - a.x) * std::cos(mid) + (b.y - a.y) * std::sin(mid)) / kControlDt;
    next.wz = (b.theta - a.theta) / kControlDt;
  }

  std::printf("## %s seed %d  T=%.1f s (cycle %ld)  behavior(trace)=%s  speed_ratio(再生)=%.3f\n", t.scenario.c_str(), t.seed,
              now, k, at(t, kSub * k).behavior.c_str(), dec.speed_ratio);
  std::printf("pose (%.3f, %.3f, %.3f)  vel(差分) v=%.3f w=%.3f  人 %zu 人(max_range 内 %zu)\n", pose.x, pose.y, pose.theta,
              vel.vx, vel.wz, people.agents.size(), near.size());
  std::printf("lim.v_max=%.3f  候補 %zu\n", lim.v_max, cands.size());
  std::printf("| vx | wz | total | obs | path | goal | speed | prox | coll | g [m] | 選択 |\n|---|---|---|---|---|---|---|---|---|---|---|\n");
  int n_g03 = 0, n_valid = 0;
  for (std::size_t i = 0; i < rows.size(); ++i) {
    const Row& r = rows[i];
    if (r.collide) { std::printf("| %.3f | %.3f | 棄却(障害物) | | | | | | | | |\n", r.c.vx, r.c.wz); continue; }
    ++n_valid;
    if (r.g >= 0.3) ++n_g03;
    std::printf("| %.3f | %.3f | %.2f | %.2f | %.2f | %.2f | %.2f | %.2f | %.2f | %.3f | %s |\n", r.c.vx, r.c.wz, r.total, r.obs,
                r.path, r.goal, r.speed, r.prox, r.coll, r.g > 1e8 ? NAN : r.g, i == best ? "*" : "");
  }
  std::printf("g >= 0.3 m の候補: %d / %d\n", n_g03, n_valid);
  if (best < rows.size())
    std::printf("照合: 自前の選択 (%.3f, %.3f)  compute() (%.3f, %.3f)  %s  trace 次周期 (%.3f, %.3f)\n", rows[best].c.vx,
                rows[best].c.wz, res.cmd.vx, res.cmd.wz,
                (std::fabs(rows[best].c.vx - res.cmd.vx) < 1e-9 && std::fabs(rows[best].c.wz - res.cmd.wz) < 1e-9) ? "一致" : "不一致(内訳の再現に誤り)",
                next.vx, next.wz);
  std::printf("照合: compute() と trace 次周期の差 dv=%.3f dw=%.3f\n\n", res.cmd.vx - next.vx, res.cmd.wz - next.wz);
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 3) { std::fprintf(stderr, "usage: cand_probe TRACE T [T ...]\n"); return 2; }
  Trace t;
  if (!load(argv[1], t)) { std::fprintf(stderr, "cannot read %s\n", argv[1]); return 2; }
  for (int i = 2; i < argc; ++i) {
    const long k = std::lround(std::atof(argv[i]) / kControlDt);
    if (k < 1 || kSub * (k + 1) > static_cast<long>(t.frames.size())) {
      std::fprintf(stderr, "T=%s は trace の範囲外(周期 1 以上、次の周期まで必要)\n", argv[i]);
      return 2;
    }
    probeAt(t, k, 0.0);
  }
  (void)kPhysicsDt;
  return 0;
}
