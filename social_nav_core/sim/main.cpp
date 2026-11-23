// social_nav_sim: ROS なしの簡易シミュレータ(design 6.2)
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <map>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "sim/metrics.hpp"
#include "sim/ped_model.hpp"
#include "sim/ttc_ped_model.hpp"
#include "sim/orca_ped_model.hpp"
#include "sim/ped_speed_mix.hpp"
#include "sim/ped_variants.hpp"
#include "sim/safety.hpp"
#include "sim/scenarios.hpp"
#include "sim/social_force.hpp"
#include "social_nav_core/footprint.hpp"
#include "social_nav_core/motion_model.hpp"
#include "social_nav_core/people_tracker.hpp"
#include "social_nav_core/sampling_controller.hpp"
#include "social_nav_core/tactical_selector.hpp"

using namespace social_nav;
using namespace social_nav::sim;

namespace {

constexpr double kPhysicsDt = 0.02;
constexpr double kControlDt = 0.1;
constexpr double kGoalRadius = 0.25;
constexpr double kSafetyClearance = 0.15;
constexpr double kRobotRadius = 0.3;
constexpr double kPedRadius = 0.3;
constexpr double kPathWindow = 4.0;
constexpr double kPathSpacing = 0.5;

struct Options {
  std::string scenario{"all"};
  int seeds{20};
  std::string controller{"social"};
  bool no_tactical{false};
  bool use_tracker{false};
  bool peds_ignore_robot{false};
  bool no_peds{false};   // --no-peds: makeScenario の後に人を空にする(屋内シナリオの対照。既定 off)
  double sim_time{0.0};   // --sim-time T: 打ち切り時間 [s] で timeout を短くする(0 は無効。再現性テスト用)
  std::string ped_model{"sfm2"};   // --ped-model {legacy,sfm2,ttc,orca}。既定は sfm2(legacy は比較用、ttc と orca は記録用)
  bool ped_variants{true};          // --ped-variants {on,off}(sfm2、ttc、orca で効く。ttc と orca は属性を読まず、人の構成だけ sfm2 と同じにする)
  bool ped_speed_mix{false};        // --ped-speed-mix {off,on}(候補 8。既定 off。on は corridor_oncoming / crossing / crowd の人に速さの種別を付ける。記録のみ)
  double v_min{0.0};   // --v-min: ControllerParams.limits.v_min に渡すだけ(後退の試験用。既定 0.0 = 後退なし)
  double prox_sigma{ProxemicsParams{}.sigma};          // --prox-sigma: SocialCostParams.prox.sigma(B1。既定 = 現行の 0.45)
  double rear_ratio{ProxemicsParams{}.rear_ratio};     // --rear-ratio: SocialCostParams.prox.rear_ratio(B2。既定 = 現行の 0.6)
  TrackerParams tracker;   // --tracker-q(指定なしは既定値)
  SafetyMode safety_mode{SafetyMode::Predictive};   // --safety-mode(既定 = 現行)
  bool diag_ped_vel_cmd{false};   // --diag-ped-robot-vel cmd(診断専用。既定 applied = 現行)
  bool tracker_q_set{false};        // --tracker-q が指定された(--use-tracker 時のみ有効)
  bool tracker_oracle_vel{false};   // --tracker-oracle-vel: 確定 track の速度だけ真値に差し替える(--use-tracker 時のみ)
  bool door_yield{false};   // --door-yield: ControllerParams.door_yield(既定 off = 現行)
  bool accel_rollout{false};   // --accel-rollout / --no-accel-rollout: ControllerParams.accel_rollout(既定 off = 現行)
  bool tie_min_turn_stopped{false};   // --tie-min-turn-stopped: ControllerParams.tie_min_turn_stopped(譲るときの 1 周の案 T2。既定 off)
  bool tie_path_heading{false};   // --tie-path-heading: ControllerParams.tie_path_heading(譲るときの 1 周の案 H。既定 off)
  bool tie_min_turn{false};   // --tie-min-turn: ControllerParams.tie_min_turn(譲るときの 1 周の案 T。既定 off = 現行)
  double w_turn{0.0};      // --w-turn: ControllerParams.w_turn(案 A。既定 0 = 現行)
  double w_heading{0.0};   // --w-heading: ControllerParams.w_heading(案 C。既定 0 = 現行)
  bool stop_aware{false};   // --stop-aware: ControllerParams.stop_aware(R2。既定 off = 現行)
  bool check{false};
  bool check_moving{false};   // --check の判定を「走行中の衝突」(M1a)だけにする
  int check_moving_max{0};    // M1a の合計の上限(--check-moving-max N。未指定は 0)
  bool check_moving_max_set{false};
  int seed_start{1};
  std::string csv;
  std::string trace;   // --trace: 1 走行の軌跡を書き出す(動画描画用)
  std::string wall_csv;   // --wall-csv: 壁距離 < kRobotRadius の周期数と all_colliding の周期数を 1 走行 1 行で書く(既定の CSV は変えない)
  bool occlusion{false};   // --occlusion: 壁の線分が視線を遮る人は、検出・tracker なしの people.agents・安全層のどれにも渡さない(既定 off = 現行)
  std::string blind_csv;   // --blind-csv: 見えた時刻・確定・接触などを 1 走行 1 行で書く(既定の CSV は変えない)
  std::string event_csv;   // --event-csv: 急に止まる・振り返る人の発火時刻・距離・発火後の最小クリアランスを 1 走行 1 行で書く(既定の CSV は変えない)
  std::string contacts_csv;   // --contacts-csv: 接触イベントを 1 行ずつ書き出す(診断用)
};

void usage() {
  std::printf(
      "usage: social_nav_sim [--scenario {corridor_oncoming,crossing,overtake,follow,static_group,crowd,all}]\n"
      "                      (後方の人流・記録のみ: rear_follow,rear_overtake,rear_overtake_flow、rear = その 3 本。all には含めない)\n"
      "                      (屋内・記録のみ: door_oncoming,side_door,group_oncoming、indoor = その 3 本。all には含めない)\n"
      "                      [--seeds N] [--controller {social,baseline}] [--no-tactical] [--use-tracker]\n"
      "                      [--peds-ignore-robot] [--no-peds] [--sim-time T] [--v-min V] [--prox-sigma S] [--rear-ratio R] [--safety-mode {predictive,cm_stop,cm,off}] [--diag-ped-robot-vel {applied,cmd}] [--ped-model {legacy,sfm2,ttc,orca}] [--ped-variants {on,off}] [--ped-speed-mix {on,off}]\n"
      "                      (見えない人・記録のみ: blind_corner、blind = その 1 本。all には含めない。--occlusion と使う)\n"
      "                      [--occlusion] [--blind-csv out.csv]\n"
      "                      (エレベーター・記録のみ: elevator_flow、elevator = その 1 本。all には含めない)\n"
      "                      (急に止まる・振り返る人・記録のみ: sudden_stop,sudden_turn,sudden_reverse、sudden = その 3 本。all には含めない)\n"
      "                      [--event-csv out.csv]\n"
      "                      [--tracker-q A] [--tracker-oracle-vel] [--stop-aware] [--tie-min-turn] [--tie-min-turn-stopped] [--tie-path-heading] [--w-turn W] [--w-heading W] [--accel-rollout|--no-accel-rollout] [--door-yield] [--csv out.csv] [--wall-csv out.csv] [--check] [--check-moving] [--check-moving-max N] [--seed-start K]\n"
      "                      [--trace file]   (--seeds 1 と単一シナリオのときだけ)\n"
      "                      [--contacts-csv file]   (接触イベントの診断出力)\n"
      "  --check-moving-max N は上限 N。後ろに --check-moving を付けても上限は 0 に戻らない\n");
}

bool parseArgs(int argc, char** argv, Options& o) {
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    auto next = [&](std::string& dst) {
      if (i + 1 >= argc) throw std::invalid_argument("missing value for " + a);
      dst = argv[++i];
    };
    std::string v;
    if (a == "--scenario") next(o.scenario);
    else if (a == "--seeds") { next(v); o.seeds = std::atoi(v.c_str()); }
    else if (a == "--controller") next(o.controller);
    else if (a == "--csv") next(o.csv);
    else if (a == "--no-tactical") o.no_tactical = true;
    else if (a == "--use-tracker") o.use_tracker = true;
    else if (a == "--peds-ignore-robot") o.peds_ignore_robot = true;
    else if (a == "--no-peds") o.no_peds = true;
    else if (a == "--sim-time") {
      next(v);
      std::size_t pos = 0;
      o.sim_time = std::stod(v, &pos);
      if (pos != v.size() || !(o.sim_time > 0.0)) throw std::invalid_argument("--sim-time must be a number > 0: " + v);
    }
    else if (a == "--ped-model") {
      next(o.ped_model);
      if (o.ped_model != "legacy" && o.ped_model != "sfm2" && o.ped_model != "ttc" && o.ped_model != "orca") throw std::invalid_argument("--ped-model must be legacy, sfm2, ttc or orca: " + o.ped_model);
    }
    else if (a == "--ped-variants") {
      next(v);
      if (v != "on" && v != "off") throw std::invalid_argument("--ped-variants must be on or off: " + v);
      o.ped_variants = (v == "on");
    }
    else if (a == "--ped-speed-mix") {
      next(v);
      if (v != "on" && v != "off") throw std::invalid_argument("--ped-speed-mix must be on or off: " + v);
      o.ped_speed_mix = (v == "on");
    }
    else if (a == "--safety-mode") {
      next(v);
      if (!parseSafetyMode(v, o.safety_mode)) throw std::invalid_argument("--safety-mode must be predictive, cm_stop, cm or off: " + v);
    }
    else if (a == "--diag-ped-robot-vel") {
      next(v);
      if (v != "applied" && v != "cmd") throw std::invalid_argument("--diag-ped-robot-vel must be applied or cmd: " + v);
      o.diag_ped_vel_cmd = (v == "cmd");
    }
    else if (a == "--v-min") {
      next(v);
      std::size_t pos = 0;
      o.v_min = std::stod(v, &pos);
      if (pos != v.size() || o.v_min > 0.0) throw std::invalid_argument("--v-min must be a number <= 0: " + v);
    }
    else if (a == "--prox-sigma" || a == "--rear-ratio") {
      next(v);
      std::size_t pos = 0;
      const double x = std::stod(v, &pos);
      if (pos != v.size() || !std::isfinite(x) || !(x > 0.0)) throw std::invalid_argument(a + " must be a finite number > 0: " + v);
      (a == "--prox-sigma" ? o.prox_sigma : o.rear_ratio) = x;
    }
    else if (a == "--tracker-q") {
      next(v);
      std::size_t pos = 0;
      const double x = std::stod(v, &pos);
      if (pos != v.size()) throw std::invalid_argument(a + " must be a number: " + v);
      if (!(x > 0.0)) throw std::invalid_argument("--tracker-q must be > 0: " + v);
      o.tracker.process_noise_acc = x;
      o.tracker_q_set = true;
    }
    else if (a == "--tracker-oracle-vel") o.tracker_oracle_vel = true;
    else if (a == "--stop-aware") o.stop_aware = true;
    else if (a == "--tie-path-heading") o.tie_path_heading = true;
    else if (a == "--tie-min-turn") o.tie_min_turn = true;
    else if (a == "--tie-min-turn-stopped") o.tie_min_turn_stopped = true;
    else if (a == "--w-turn") { next(v); o.w_turn = std::atof(v.c_str()); }
    else if (a == "--w-heading") { next(v); o.w_heading = std::atof(v.c_str()); }
    else if (a == "--accel-rollout") o.accel_rollout = true;
    else if (a == "--no-accel-rollout") o.accel_rollout = false;
    else if (a == "--door-yield") o.door_yield = true;
    else if (a == "--check") o.check = true;
    else if (a == "--check-moving") { o.check = true; o.check_moving = true; }
    else if (a == "--check-moving-max") {
      next(v);
      std::size_t pos = 0;
      const int n = std::stoi(v, &pos);
      if (pos != v.size() || n < 0) throw std::invalid_argument("--check-moving-max must be an integer >= 0: " + v);
      o.check = true; o.check_moving = true; o.check_moving_max = n; o.check_moving_max_set = true;
    }
    else if (a == "--trace") next(o.trace);
    else if (a == "--contacts-csv") next(o.contacts_csv);
    else if (a == "--wall-csv") next(o.wall_csv);
    else if (a == "--blind-csv") next(o.blind_csv);
    else if (a == "--event-csv") next(o.event_csv);
    else if (a == "--occlusion") o.occlusion = true;
    else if (a == "--seed-start") { next(v); o.seed_start = std::atoi(v.c_str()); }
    else if (a == "--help" || a == "-h") return false;
    else throw std::invalid_argument("unknown argument: " + a);
  }
  if (o.seeds < 1) throw std::invalid_argument("--seeds must be >= 1");
  if (o.controller != "social" && o.controller != "baseline")
    throw std::invalid_argument("--controller must be social or baseline");
  if (o.tracker_q_set && !o.use_tracker)
    throw std::invalid_argument("--tracker-q requires --use-tracker");
  if (o.tracker_oracle_vel && !o.use_tracker)
    throw std::invalid_argument("--tracker-oracle-vel requires --use-tracker");
  if (!o.trace.empty() && (o.seeds != 1 || o.scenario == "all" || o.scenario == "rear" || o.scenario == "indoor" || o.scenario == "blind" || o.scenario == "sudden" || o.scenario == "elevator"))
    throw std::invalid_argument("--trace requires --seeds 1 and a single --scenario");
  return true;
}

std::vector<Agent> toAgents(const std::vector<Pedestrian>& peds) {
  std::vector<Agent> out;
  out.reserve(peds.size());
  for (const Pedestrian& p : peds) {
    Agent a;
    a.id = p.id;
    a.pos = p.pos;
    a.vel = p.vel;
    a.pos_cov = {0.01, 0.0, 0.01};
    a.radius = p.radius;
    a.confidence = 1.0;
    out.push_back(a);
  }
  return out;
}

// 直線経路のうち、ロボットの射影点から kPathWindow 先までを kPathSpacing 間隔で切り出す
std::vector<Pose2D> pathWindow(const Pose2D& start, Vec2 goal, const Pose2D& robot) {
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

// 障害物コスト(インフレーション済み costmap 相当)。中心が障害物から kRobotRadius+0.05 未満なら衝突、
// 外側は指数で減衰(1 未満)。agents が非 null なら人も障害物として扱う(baseline 用。social は人を社会的コストで扱う)
ObstacleCostFn makeObstacleFn(const std::vector<Segment>& walls, const std::vector<Agent>* agents) {
  return [&walls, agents](double x, double y) {
    double d = distanceToWalls({x, y}, walls);
    if (agents) {
      for (const Agent& a : *agents) d = std::min(d, std::hypot(x - a.pos.x, y - a.pos.y) - a.radius);
    }
    if (d < kRobotRadius + 0.05) return 1.0;
    return 0.9 * std::exp(-(d - kRobotRadius - 0.05) / 0.25);
  };
}

struct WallStats { int wall_cycles{0}, all_colliding_cycles{0}; };

// --blind-csv の 1 走行分(壁の遮蔽)。時刻は -1 = 該当なし。時刻の規則は docs/sim_results.md の「壁による遮蔽」節
struct BlindStats {
  bool has_ped{false};
  int side{0}, branch{-1};
  double w{-1}, v0{-1};
  double t_first_los{-1}, d_first_los{-1}, speed_first_los{-1};
  double t_first_det{-1}, t_confirm{-1}, t_first_safety_stop{-1};
  double t_contact{-1}, speed_contact{-1};
  double min_clear{1e9}, t_min_clear{-1};
};

// --event-csv の 1 走行分(急に止まる・振り返る人)。人 0 番の事象。時刻・距離は -1 = 発火せず。min_clear は発火後の外形間の最小クリアランス(1e9 = なし)
struct EventStats {
  bool has_event{false};
  int kind{0};
  double trigger_d{-1}, t_fire{-1}, d_fire{-1};
  double min_clear{1e9}, t_min_clear{-1};
};

RunMetrics runOne(const Options& o, const std::string& scenario, int seed, std::vector<ContactEvent>* contacts, WallStats* wall = nullptr, BlindStats* blind = nullptr, EventStats* event = nullptr) {
  std::mt19937 rng(static_cast<unsigned>(seed));
  std::mt19937 sense_rng(static_cast<unsigned>(seed) + 7777u);
  Scenario sc = makeScenario(scenario, rng);
  const bool sfm2 = o.ped_model == "sfm2";
  const bool ttc = o.ped_model == "ttc";
  const bool orca = o.ped_model == "orca";
  const uint32_t ped_seed = static_cast<uint32_t>(seed) + 4242u;   // 歩行者専用 engine(rng・sense_rng とは別)
  if (o.no_peds) sc.peds.clear();
  if (o.sim_time > 0.0 && o.sim_time < sc.timeout) sc.timeout = o.sim_time;
  if ((sfm2 || ttc || orca) && o.ped_variants) applyPedVariants(sc, static_cast<uint32_t>(seed) + 4243u);   // 付与用は雑音と別の列
  if (o.ped_speed_mix) applySpeedMix(sc, static_cast<uint32_t>(seed) + 8647u);   // 候補 8。専用 engine(配置は off と同じ)

  if (blind && !sc.peds.empty()) {
    const PedInit& b = sc.peds[0];
    blind->has_ped = true;
    blind->side = b.pos.y >= 0.0 ? 1 : -1;
    blind->branch = !b.has_via ? 0 : (b.goal.x < b.pos.x ? 1 : 2);
    blind->w = sc.opening_w;
    blind->v0 = b.v0;
  }

  std::vector<Pedestrian> peds;
  for (std::size_t i = 0; i < sc.peds.size(); ++i) {
    const PedInit& pi = sc.peds[i];
    const Vec2 first_goal = pi.has_via ? pi.via : pi.goal;   // via があればまず via へ(屋内シナリオ)
    Pedestrian p;
    p.id = static_cast<uint32_t>(i + 1);
    p.pos = pi.pos;
    p.home = pi.pos;
    p.goal = first_goal;
    p.v0 = pi.v0;
    p.radius = kPedRadius;
    p.frozen = pi.frozen || pi.release_t > 0.0;   // release_t > 0 の人は、物理ループで解除されるまで止まる
    p.loop = pi.loop;
    p.kind = pi.kind;
    p.group_id = pi.group_id;
    const double gd = std::hypot(first_goal.x - pi.pos.x, first_goal.y - pi.pos.y);
    if (!pi.frozen && !(pi.release_t > 0.0) && gd > 1e-9) p.vel = {pi.v0 * (first_goal.x - pi.pos.x) / gd, pi.v0 * (first_goal.y - pi.pos.y) / gd};
    peds.push_back(p);
  }
  std::vector<PedEventState> ev_state(peds.size());   // 事象の状態(kind 0 の人は何もしない)
  if (event && !sc.peds.empty() && sc.peds[0].event.kind != 0) {
    event->has_event = true;
    event->kind = sc.peds[0].event.kind;
    event->trigger_d = sc.peds[0].event.trigger_d;
  }
  std::vector<PedPauseState> pause_state(peds.size());   // 周期停止の状態(period 0 の人は何もしない)
  std::vector<char> via_done(peds.size(), 0);   // via を通過して最終目標へ切り替え済み(has_via の人だけ使う)
  const SocialForceWorld world(SfmParams{}, sc.walls);
  PedModel ped_model(PedParams{}, sc.walls, ped_seed);
  TtcPedModel ttc_model(TtcParams{}, sc.walls, ped_seed);   // ttc のときだけ使う(同じ seed の専用 engine)
  const OrcaPedModel orca_model(OrcaParams{}, sc.walls);    // orca のときだけ使う(決定的。乱数なし)

  FootprintSpec fspec;
  fspec.radius = kRobotRadius;
  std::shared_ptr<const Footprint> fp = makeFootprint(fspec);
  std::shared_ptr<const MotionModel> model = makeMotionModel("diff_drive");
  ControllerParams cp;
  cp.w_speed = 1.0;   // 重み調整 1 回(既定 0.2 → 1.0)。停止からの再発進で近接ペナルティに負けて動けなくなる走行を減らす。docs/sim_results.md 参照
  if (o.controller == "baseline") cp.w_social = 0.0;
  cp.limits.v_min = o.v_min;
  cp.stop_aware = o.stop_aware;
  cp.tie_min_turn = o.tie_min_turn;
  cp.tie_path_heading = o.tie_path_heading;
  cp.tie_min_turn_stopped = o.tie_min_turn_stopped;
  cp.w_turn = o.w_turn;
  cp.w_heading = o.w_heading;
  cp.accel_rollout = o.accel_rollout;
  cp.door_yield = o.door_yield;
  cp.social.prox.sigma = o.prox_sigma;
  cp.social.prox.rear_ratio = o.rear_ratio;
  SamplingController controller(cp, model, fp);
  TacticalSelector tactical{TacticalParams{}};
  PeopleTracker tracker{o.tracker};
  MetricsRecorder rec(fp);
  if (contacts) { rec.enableContactLog(); controller.setDiagnostics(true); }
  const bool baseline = o.controller == "baseline";
  TrackedPeople people;   // 毎周期更新。baseline の障害物コストが参照する
  const ObstacleCostFn obstacles = makeObstacleFn(sc.walls, baseline ? &people.agents : nullptr);
  const ObstacleCostFn static_obstacles = makeObstacleFn(sc.walls, nullptr);   // door_yield の狭所点用(人を含まない)

  Pose2D pose = sc.start;
  Twist2D vel;
  const double straight = std::hypot(sc.goal.x - sc.start.x, sc.goal.y - sc.start.y);
  std::normal_distribution<double> noise(0.0, 0.05);
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  const int substeps = static_cast<int>(std::lround(kControlDt / kPhysicsDt));
  const long n_cycles = static_cast<long>(std::lround(sc.timeout / kControlDt));

  std::ofstream trace;  // 物理ステップごと(50 Hz)に書く。描画側が時刻で 20 fps に再サンプルする
  if (!o.trace.empty()) {
    trace.open(o.trace);
    if (!trace) throw std::runtime_error("cannot open " + o.trace);
    trace << std::fixed << std::setprecision(3);
    trace << "H " << scenario << ' ' << seed << ' ' << o.controller << '\n';
    for (const Segment& w : sc.walls) trace << "W " << w.a.x << ' ' << w.a.y << ' ' << w.b.x << ' ' << w.b.y << '\n';
    trace << "S " << sc.start.x << ' ' << sc.start.y << '\n' << "G " << sc.goal.x << ' ' << sc.goal.y << '\n';
    trace << "R " << kRobotRadius << ' ' << kPedRadius << '\n';
    for (const Pedestrian& p : peds) trace << "P " << p.id << ' ' << p.kind << ' ' << p.group_id << '\n';
    if (o.ped_speed_mix && std::any_of(sc.peds.begin(), sc.peds.end(), [](const PedInit& q) { return q.speed_type != 0; }))
      for (std::size_t i = 0; i < sc.peds.size(); ++i) trace << "V " << peds[i].id << ' ' << sc.peds[i].speed_type << '\n';   // 速さの種別(対象のシナリオの on のときだけ。off と対象外の trace は従来と同じ)
  }

  bool reached = false;
  double reach_time = sc.timeout;
  for (long k = 0; k < n_cycles && !reached; ++k) {
    const double t = static_cast<double>(k) * kControlDt;
    const std::vector<Agent> truth = toAgents(peds);

    if (blind && blind->has_ped && blind->t_first_los < 0 && lineOfSight({pose.x, pose.y}, truth[0].pos, sc.walls)) {
      blind->t_first_los = t;
      blind->d_first_los = std::hypot(truth[0].pos.x - pose.x, truth[0].pos.y - pose.y);
      blind->speed_first_los = std::hypot(vel.vx, vel.vy);
    }
    people = TrackedPeople{};
    people.frame_id = "map";
    people.stamp = t;
    if (o.use_tracker) {
      std::vector<Detection> dets;
      for (const Agent& a : truth) {
        if (std::hypot(a.pos.x - pose.x, a.pos.y - pose.y) > 10.0) continue;   // 視野距離
        if (unit(sense_rng) > 0.9) continue;                                   // 検出率
        Detection d;
        d.pos = {a.pos.x + noise(sense_rng), a.pos.y + noise(sense_rng)};
        if (o.occlusion && !lineOfSight({pose.x, pose.y}, a.pos, sc.walls)) continue;   // 壁の裏は捨てる(雑音は引いた後。sense_rng の消費は off と同じ)
        d.cov = {0.05 * 0.05, 0.0, 0.05 * 0.05};
        d.radius = a.radius;
        d.source = DetectionSource::LIDAR;
        dets.push_back(d);
      }
      if (blind && !dets.empty() && blind->t_first_det < 0) blind->t_first_det = t;
      tracker.update(dets, t);
      people = tracker.confirmed("map");
      rec.recordTracks(truth, people);   // M13・接触 CSV の track 列は tracker の推定速度のまま(差し替え前)
      if (o.tracker_oracle_vel) {   // 位置・ID・確定遅れは tracker のまま、速度だけ M13 と同じ対応の真値に差し替える
        for (const TrackMatch& m : rec.trackMatches())
          for (Agent& t : people.agents)
            if (t.id == m.track_id) t.vel = m.truth_vel;
      }
    } else {
      for (const Agent& a : truth)
        if (!o.occlusion || lineOfSight({pose.x, pose.y}, a.pos, sc.walls)) people.agents.push_back(a);
      if (blind && blind->t_first_det < 0 && blind->t_first_los >= 0) blind->t_first_det = blind->t_first_los;
    }
    if (blind && !people.agents.empty() && blind->t_confirm < 0) blind->t_confirm = t;

    const std::vector<Pose2D> path = pathWindow(sc.start, sc.goal, pose);
    TacticalDecision dec;
    if (!o.no_tactical) {
      TacticalInput in;
      in.now = t;
      in.robot = pose;
      in.path = &path;
      in.people = &people;
      dec = tactical.step(in);
    }
    rec.recordBehavior(dec.behavior, kControlDt);

    const auto t0 = std::chrono::steady_clock::now();
    const ControlResult res = controller.compute(pose, vel, path, people, obstacles, dec.speed_ratio, static_obstacles);
    const auto t1 = std::chrono::steady_clock::now();
    rec.recordControlTime(std::chrono::duration<double, std::milli>(t1 - t0).count());
    const Twist2D cmd = res.all_colliding ? Twist2D{} : res.cmd;
    if (wall) {
      if (distanceToWalls({pose.x, pose.y}, sc.walls) < kRobotRadius) ++wall->wall_cycles;
      if (res.all_colliding) ++wall->all_colliding_cycles;
    }
    rec.setControlContext(std::hypot(cmd.vx, cmd.vy), dec.speed_ratio, res.diag, cmd.vx);
    if (trace.is_open() && o.door_yield) trace << "D " << t << ' ' << res.door_narrow_s << ' ' << (res.door_yield_active ? 1 : 0) << '\n';   // door_yield: 時刻 狭所点までの距離(なし -1) 発動

    for (int s = 0; s < substeps && !reached; ++s) {
      // 安全層。predictive(現行): 指令を適用すると外形間クリアランスが 0.15 m 未満かつ縮む場合は強制停止。
      // cm_stop / cm: ROS の collision_monitor と同じ静的な円(現在の外形間距離。向きも縮みも見ない)。off は診断専用
      Twist2D applied = cmd;
      bool stop = false;
      if (o.safety_mode == SafetyMode::Predictive) {
        const Pose2D next = model->integrate(pose, cmd, kPhysicsDt);
        for (const Pedestrian& p : peds) {
          if (o.occlusion && !visibleAny({pose.x, pose.y}, p.pos, p.radius, sc.walls)) continue;   // 壁の裏の人は安全層にも見えない
          const double c_now = fp->clearance(pose, p.pos) - p.radius;
          const double c_next = fp->clearance(next, {p.pos.x + p.vel.x * kPhysicsDt, p.pos.y + p.vel.y * kPhysicsDt}) - p.radius;
          if (c_next < kSafetyClearance && c_next < c_now) { stop = true; break; }
        }
      } else if (o.safety_mode != SafetyMode::Off) {
        double min_clear = 1e9;
        for (const Pedestrian& p : peds) {
          if (o.occlusion && !visibleAny({pose.x, pose.y}, p.pos, p.radius, sc.walls)) continue;
          min_clear = std::min(min_clear, fp->clearance(pose, p.pos) - p.radius);
        }
        const CmOutcome cm = cmSafety(o.safety_mode, min_clear);
        stop = cm.stop;
        applied.vx *= cm.scale; applied.vy *= cm.scale; applied.wz *= cm.scale;
      }
      if (stop) applied = Twist2D{};
      pose = model->integrate(pose, applied, kPhysicsDt);
      vel = applied;
      for (std::size_t i = 0; i < peds.size(); ++i) {   // release_t の解除(候補 6): t >= release_t の最初のステップで目標向きの速度 v0 を与える
        const PedInit& pi = sc.peds[i];
        if (pi.release_t > 0.0 && peds[i].frozen && t + s * kPhysicsDt >= pi.release_t) {
          peds[i].frozen = false;
          const Vec2 g = pi.has_via ? pi.via : pi.goal;
          const double gd = std::hypot(g.x - peds[i].pos.x, g.y - peds[i].pos.y);
          if (gd > 1e-9) peds[i].vel = {pi.v0 * (g.x - peds[i].pos.x) / gd, pi.v0 * (g.y - peds[i].pos.y) / gd};
        }
      }
      for (std::size_t i = 0; i < peds.size(); ++i) {   // via の切り替え: via から 0.5 m 以内、または via を越えた((pos-via)·(via-出発点) > 0)
        const PedInit& pi = sc.peds[i];
        if (!pi.has_via || via_done[i]) continue;
        const double px = peds[i].pos.x - pi.via.x, py = peds[i].pos.y - pi.via.y;
        // 「越えた」は出発点から via への向きで見る(via から goal への向きだと、合流で直角に曲がるとき雑音の符号で早く切り替わる)
        if (std::hypot(px, py) < 0.5 || px * (pi.via.x - pi.pos.x) + py * (pi.via.y - pi.pos.y) > 0.0) {
          via_done[i] = 1;
          peds[i].goal = pi.goal;
        }
      }
      for (std::size_t i = 0; i < peds.size(); ++i)   // 急に止まる・振り返る人。v0 と goal だけ差し替える
        if (sc.peds[i].event.kind != 0) stepPedEvent(sc.peds[i].event, ev_state[i], peds[i], {pose.x, pose.y}, t + (s + 1) * kPhysicsDt, kPhysicsDt);
      for (std::size_t i = 0; i < peds.size(); ++i)   // 周期停止(候補 8)。v0 だけ差し替える
        if (sc.peds[i].pause.period > 0.0) stepPedPause(sc.peds[i].pause, pause_state[i], peds[i], t + (s + 1) * kPhysicsDt, kPhysicsDt);
      const Twist2D& seen = o.diag_ped_vel_cmd ? cmd : applied;   // 人に見せるロボット速度(cmd は診断専用)
      RobotBody body{{pose.x, pose.y}, {seen.vx * std::cos(pose.theta), seen.vx * std::sin(pose.theta)}, kRobotRadius};
      if (orca) orca_model.step(peds, o.peds_ignore_robot ? nullptr : &body, kPhysicsDt);
      else if (ttc) ttc_model.step(peds, o.peds_ignore_robot ? nullptr : &body, kPhysicsDt);
      else if (sfm2) ped_model.step(peds, o.peds_ignore_robot ? nullptr : &body, kPhysicsDt);
      else world.step(peds, o.peds_ignore_robot ? nullptr : &body, kPhysicsDt);
      rec.recordPhysics(pose, applied, toAgents(peds), stop, kPhysicsDt);
      if (event && event->has_event && ev_state[0].phase != 0) {
        if (event->t_fire < 0) { event->t_fire = ev_state[0].t_fire; event->d_fire = ev_state[0].d_fire; }
        const double c = fp->clearance(pose, peds[0].pos) - peds[0].radius;
        if (c < event->min_clear) { event->min_clear = c; event->t_min_clear = t + (s + 1) * kPhysicsDt; }
      }
      if (blind) {
        const double ts = t + (s + 1) * kPhysicsDt;
        if (stop && blind->t_first_los >= 0 && blind->t_first_safety_stop < 0) blind->t_first_safety_stop = ts;
        for (const Pedestrian& p : peds) {
          const double c = fp->clearance(pose, p.pos) - p.radius;
          if (c < blind->min_clear) { blind->min_clear = c; blind->t_min_clear = ts; }
          if (c < 0.0 && blind->t_contact < 0) { blind->t_contact = ts; blind->speed_contact = std::hypot(applied.vx, applied.vy); }
        }
      }
      if (trace.is_open()) {
        // F 時刻 x y theta ふるまい 人数 (x y)*人数
        trace << "F " << t + (s + 1) * kPhysicsDt << ' ' << pose.x << ' ' << pose.y << ' ' << pose.theta << ' ' << toString(dec.behavior)
              << ' ' << peds.size();
        for (const Pedestrian& p : peds) trace << ' ' << p.pos.x << ' ' << p.pos.y;
        trace << '\n';
      }
      if (std::hypot(pose.x - sc.goal.x, pose.y - sc.goal.y) <= kGoalRadius) {
        reached = true;
        reach_time = t + (s + 1) * kPhysicsDt;
      }
    }
  }
  if (contacts) *contacts = rec.contacts();
  return rec.finish(reached, reach_time, straight);
}

struct Row { std::string scenario; int seed; RunMetrics m; std::vector<ContactEvent> contacts; WallStats wall; BlindStats blind; EventStats event; };

void writeContactsCsv(const std::string& path, const std::vector<Row>& rows) {
  std::ofstream f(path);
  // track_vel_err は tracker の推定速度の誤差(--tracker-oracle-vel の差し替え前の値)
  f << "scenario,seed,t,ped,cls,robot_speed,cmd_speed,cmd_vx,speed_ratio,stopped_dur,hyst10,hyst15,hyst20,safety_stop,"
       "safety_frac_1s,neg_cmd_frac_1s,neg_applied_frac_1s,max_speed_1s,behavior,behavior_age,close_robot,close_ped,approach_robot_1s,approach_ped_1s,"
       "ped_speed,bearing,prev_gap,steps,min_clear,"
       "d1_valid,d1_ncand,d1_nfree,d1_zero_free,d1_chosen_contact,d05_valid,d05_ncand,d05_nfree,d05_zero_free,d05_chosen_contact,track_has,track_vel_err,track_m13_class,track_age\n";
  for (const Row& r : rows)
    for (const ContactEvent& e : r.contacts)
      f << r.scenario << ',' << r.seed << ',' << e.t << ',' << e.ped_id << ',' << e.cls << ',' << e.robot_speed << ','
        << e.cmd_speed << ',' << e.cmd_vx << ',' << e.speed_ratio << ',' << e.stopped_dur << ',' << e.hyst_dur[0] << ',' << e.hyst_dur[1] << ','
        << e.hyst_dur[2] << ',' << (e.safety_stop ? 1 : 0) << ',' << e.safety_frac_1s << ',' << e.neg_cmd_frac_1s << ',' << e.neg_applied_frac_1s << ',' << e.max_speed_1s << ','
        << toString(static_cast<Behavior>(e.behavior)) << ',' << e.behavior_age << ',' << e.close_robot << ',' << e.close_ped << ','
        << e.approach_robot_1s << ',' << e.approach_ped_1s << ',' << e.ped_speed << ',' << e.bearing << ',' << e.prev_gap << ','
        << e.steps << ',' << e.min_clear << ','
        << e.diag_1s.valid << ',' << e.diag_1s.n_cand << ',' << e.diag_1s.n_free << ',' << e.diag_1s.zero_free << ',' << e.diag_1s.chosen_contact << ','
        << e.diag_05s.valid << ',' << e.diag_05s.n_cand << ',' << e.diag_05s.n_free << ',' << e.diag_05s.zero_free << ',' << e.diag_05s.chosen_contact << ','
        << (e.track.has_track ? 1 : 0) << ',' << e.track.vel_err << ',' << e.track.m13_class << ',' << e.track.track_age << '\n';
}

void writeCsv(const std::string& path, const std::vector<Row>& rows) {
  std::ofstream f(path);
  f << "scenario,seed,M1,M1_moving,M1_stopped,M1a_fast,M1a_recent,M2,M3,M4,M5,reached,M7,M8,M9,M10_proceed,M10_follow,M10_yield,M10_wait,M11,M12_mean_ms,"
       "M12_max_ms,M13_rms,M14,"
       "M13_transient_sumsq,M13_transient_n,M13_switch_sumsq,M13_switch_n,M13_accel_sumsq,M13_accel_n,"
       "M13_steady_sumsq,M13_steady_n,dup_steps,ped_ped_contacts,"
       "M4_proceed,M4_follow,M4_yield,M4_wait,M4_front,M4_side,M4_back,M4_nodir\n";
  for (const Row& r : rows) {
    const RunMetrics& m = r.m;
    f << r.scenario << ',' << r.seed << ',' << m.m1_collisions << ',' << m.m1_moving << ',' << m.m1_stopped << ',' << m.m1_moving_fast << ',' << m.m1_moving_recent << ',' << m.m2_min_clearance << ','
      << m.m3_intimate_time << ',' << m.m4_personal_time << ',' << m.m5_reach_time << ',' << (m.reached ? 1 : 0) << ','
      << m.m7_path_ratio << ',' << m.m8_stopped_time << ',' << m.m9_transitions << ',' << m.m10_dwell[0] << ','
      << m.m10_dwell[1] << ',' << m.m10_dwell[2] << ',' << m.m10_dwell[3] << ',' << m.m11_safety_stops << ','
      << m.m12_mean_ms << ',' << m.m12_max_ms << ',' << m.m13_rms << ',' << m.m14_id_switches;
    for (int c = 0; c < 4; ++c) f << ',' << m.m13_class_sumsq[c] << ',' << m.m13_class_count[c];
    f << ',' << m.dup_steps << ',' << m.ped_ped_contacts;
    for (double v : m.m4_by_behavior) f << ',' << v;
    for (double v : m.m4_by_bearing) f << ',' << v;
    f << '\n';
  }
}

void printTable(const std::vector<std::string>& scenarios, const std::vector<Row>& rows) {
  std::printf("%-18s %4s | %-9s %-7s | %-6s %-6s | %-6s %-6s | %-5s %-5s | %-5s %-5s | %-5s | %-5s | %-9s | %-6s %-7s %-7s | %-6s %-3s\n",
              "scenario", "runs", "M1 sum(mv/st)", "M2 min", "M3 mn", "M3 mx", "M4 mn", "M4 mx", "M5 mn", "M5 mx", "M6 %",
              "M7 mn", "M8 mn", "M9 mn", "M9 mx", "M11 mn", "M12 mn", "M12 mx", "M13", "M14");
  for (const std::string& sn : scenarios) {
    int n = 0, c = 0, cmv = 0, to = 0, tr_max = 0, idsw = 0;
    double m11 = 0, m12 = 0, m12x = 0;
    double m2 = 1e9, m3 = 0, m3x = 0, m4 = 0, m4x = 0, m5 = 0, m5x = 0, m7 = 0, m8 = 0, tr = 0, ss = 0;
    long cnt = 0;
    for (const Row& r : rows) {
      if (r.scenario != sn) continue;
      const RunMetrics& m = r.m;
      ++n;
      c += m.m1_collisions;
      cmv += m.m1_moving;
      m2 = std::min(m2, m.m2_min_clearance);
      m3 += m.m3_intimate_time; m3x = std::max(m3x, m.m3_intimate_time);
      m4 += m.m4_personal_time; m4x = std::max(m4x, m.m4_personal_time);
      m5 += m.m5_reach_time; m5x = std::max(m5x, m.m5_reach_time);
      if (!m.reached) ++to;
      m7 += m.m7_path_ratio;
      m8 += m.m8_stopped_time;
      tr += m.m9_transitions; tr_max = std::max(tr_max, m.m9_transitions);
      ss += m.m13_sumsq; cnt += m.m13_count;
      idsw += m.m14_id_switches;
      m11 += m.m11_safety_stops;
      m12 += m.m12_mean_ms; m12x = std::max(m12x, m.m12_max_ms);
    }
    if (n == 0) continue;
    char m1buf[32];
    std::snprintf(m1buf, sizeof m1buf, "%d(%d/%d)", c, cmv, c - cmv);
    const double rms = cnt > 0 ? std::sqrt(ss / static_cast<double>(cnt)) : 0.0;
    std::printf("%-18s %4d | %-16s %-7.3f | %-6.2f %-6.2f | %-6.2f %-6.2f | %-5.1f %-5.1f | %-5.1f %-5.2f | %-5.1f | %-5.1f | %-9d | %-6.2f %-7.3f %-7.2f | %-6.3f %-3.1f\n",
                sn.c_str(), n, m1buf, m2, m3 / n, m3x, m4 / n, m4x, m5 / n, m5x, 100.0 * to / n, m7 / n, m8 / n,
                tr / n, tr_max, m11 / n, m12 / n, m12x, rms, static_cast<double>(idsw) / n);
  }
}

}  // namespace

int main(int argc, char** argv) {
  Options o;
  try {
    if (!parseArgs(argc, argv, o)) { usage(); return 0; }
    std::vector<std::string> scenarios;
    if (o.scenario == "all") scenarios = scenarioNames();
    else if (o.scenario == "rear") scenarios = rearScenarioNames();   // 後方からの人流: 記録のみ。all には含めない
    else if (o.scenario == "indoor") scenarios = indoorScenarioNames();   // 屋内: 記録のみ。all には含めない
    else if (o.scenario == "blind") scenarios = blindScenarioNames();   // 壁の遮蔽: 記録のみ。all には含めない
    else if (o.scenario == "sudden") scenarios = suddenScenarioNames();   // 急に止まる・振り返る人: 記録のみ。all には含めない
    else if (o.scenario == "elevator") scenarios = elevatorScenarioNames();   // エレベーター: 記録のみ。all には含めない
    else scenarios.push_back(o.scenario);

    std::vector<Row> rows;
    for (const std::string& sn : scenarios)
      for (int seed = o.seed_start; seed < o.seed_start + o.seeds; ++seed) {
        Row row{sn, seed, {}, {}, {}, {}, {}};
        row.m = runOne(o, sn, seed, o.contacts_csv.empty() ? nullptr : &row.contacts, o.wall_csv.empty() ? nullptr : &row.wall,
                       o.blind_csv.empty() ? nullptr : &row.blind, o.event_csv.empty() ? nullptr : &row.event);
        rows.push_back(std::move(row));
      }

    printTable(scenarios, rows);
    if (!o.csv.empty()) writeCsv(o.csv, rows);
    if (!o.contacts_csv.empty()) writeContactsCsv(o.contacts_csv, rows);
    if (!o.wall_csv.empty()) {
      std::ofstream f(o.wall_csv);
      if (!f) throw std::runtime_error("cannot write " + o.wall_csv);
      f << "scenario,seed,wall_cycles,all_colliding_cycles\n";
      for (const Row& r : rows) f << r.scenario << ',' << r.seed << ',' << r.wall.wall_cycles << ',' << r.wall.all_colliding_cycles << '\n';
    }

    if (!o.blind_csv.empty()) {
      std::ofstream f(o.blind_csv);
      if (!f) throw std::runtime_error("cannot write " + o.blind_csv);
      f << std::fixed << std::setprecision(3);
      f << "scenario,seed,side,branch,w,v0,t_first_los,d_first_los,robot_speed_first_los,t_first_det,t_confirm,t_first_safety_stop,"
           "t_contact,robot_speed_contact,min_clear,t_min_clear\n";
      for (const Row& r : rows) {
        const BlindStats& b = r.blind;
        f << r.scenario << ',' << r.seed << ',';
        if (!b.has_ped) { f << "-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1\n"; continue; }
        f << b.side << ',' << b.branch << ',' << b.w << ',' << b.v0 << ',' << b.t_first_los << ',' << b.d_first_los << ',' << b.speed_first_los << ','
          << b.t_first_det << ',' << b.t_confirm << ',' << b.t_first_safety_stop << ',' << b.t_contact << ',' << b.speed_contact << ','
          << (b.t_min_clear < 0 ? -1.0 : b.min_clear) << ',' << b.t_min_clear << '\n';
      }
    }

    if (!o.event_csv.empty()) {
      std::ofstream f(o.event_csv);
      if (!f) throw std::runtime_error("cannot write " + o.event_csv);
      f << std::fixed << std::setprecision(3);
      f << "scenario,seed,kind,trigger_d,t_fire,d_fire,min_clear_after,t_min_clear_after\n";
      for (const Row& r : rows) {
        const EventStats& e = r.event;
        f << r.scenario << ',' << r.seed << ',';
        if (!e.has_event) { f << "0,-1,-1,-1,-1,-1\n"; continue; }
        f << e.kind << ',' << e.trigger_d << ',' << e.t_fire << ',' << e.d_fire << ',' << (e.t_min_clear < 0 ? -1.0 : e.min_clear) << ',' << e.t_min_clear << '\n';
      }
    }

    if (o.check) {
      int total = 0;
      for (const Row& r : rows) total += o.check_moving ? r.m.m1_moving : r.m.m1_collisions;
      if (total > o.check_moving_max) {
        if (o.check_moving_max_set)
          std::fprintf(stderr, "check failed: M1 = %d moving collisions (max %d)\n", total, o.check_moving_max);
        else
          std::fprintf(stderr, "check failed: M1 = %d %scollisions\n", total, o.check_moving ? "moving " : "");
        return 1;
      }
    }
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    usage();
    return 2;
  }
  return 0;
}
