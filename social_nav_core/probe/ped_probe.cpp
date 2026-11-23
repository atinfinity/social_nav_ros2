// ped_probe: 歩行者モデル(legacy / sfm2)をロボットなしで測る(妥当性の帯の確認用)。ROS 非依存。
// seed は 1001-1020 だけを使う(判定用 seed 21-120 と選定用 1-20 は使わない)。
// 置き場所は sim/ の外(sim の GLOB に入れない)。ctest には入れず、tools/run_ped_probe.sh で実行する。
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <random>
#include <string>
#include <vector>

#include "sim/ped_model.hpp"
#include "sim/social_force.hpp"

using namespace social_nav;
using namespace social_nav::sim;

namespace {

constexpr double kDt = 0.02;
constexpr double kR = 0.3;
constexpr int kSeed0 = 1001, kSeed1 = 1020;

struct Model {
  bool sfm2;
  SocialForceWorld legacy;
  PedModel sf;
  Model(bool s, const std::vector<Segment>& walls, uint32_t seed)
      : sfm2(s), legacy(SfmParams{}, walls), sf(PedParams{}, walls, seed + 4242u) {}
  void step(std::vector<Pedestrian>& p) {
    if (sfm2) sf.step(p, nullptr, kDt);
    else legacy.step(p, nullptr, kDt);
  }
};

double uni(std::mt19937& r, double lo, double hi) { return std::uniform_real_distribution<double>(lo, hi)(r); }
double walkSpeed(std::mt19937& r) { return std::max(0.5, std::normal_distribution<double>(1.3, 0.2)(r)); }

Pedestrian mk(uint32_t id, Vec2 pos, Vec2 goal, double v0) {
  Pedestrian p;
  p.id = id; p.pos = pos; p.home = pos; p.goal = goal; p.v0 = v0;
  const double d = std::hypot(goal.x - pos.x, goal.y - pos.y);
  p.vel = {v0 * (goal.x - pos.x) / d, v0 * (goal.y - pos.y) / d};
  return p;
}

double mean(const std::vector<double>& v) {
  double s = 0; for (double x : v) s += x;
  return v.empty() ? 0.0 : s / static_cast<double>(v.size());
}
double vmin(const std::vector<double>& v) { return v.empty() ? 0.0 : *std::min_element(v.begin(), v.end()); }
double vmax(const std::vector<double>& v) { return v.empty() ? 0.0 : *std::max_element(v.begin(), v.end()); }

std::vector<Segment> longCorridor(double half_w) {
  return {{{-100.0, half_w}, {300.0, half_w}}, {{-100.0, -half_w}, {300.0, -half_w}}};
}

// ---- 1. 自由流: 空の通路で 1 人。平均速度 / v0
double freeFlowRatio(bool sfm2, int seed, int kind) {
  std::mt19937 rng(static_cast<unsigned>(seed));
  const double v0 = 1.3;
  (void)rng;
  Model m(sfm2, {}, static_cast<uint32_t>(seed));
  std::vector<Pedestrian> peds{mk(1, {0, 0}, {1000, 0}, v0)};
  peds[0].kind = kind;
  if (kind == 2) peds[0].attention = 1.0;
  double sum = 0; int cnt = 0;
  for (int k = 0; k < 2000; ++k) {   // 40 s
    m.step(peds);
    if (k >= 500) { sum += std::hypot(peds[0].vel.x, peds[0].vel.y); ++cnt; }
  }
  return sum / cnt / v0;
}

// ---- 周期境界の通路(長さ L、幅 W)。ghost を毎ステップ作り直し、固定サイズ(N + N)で step する(雑音の状態が保たれる)
struct Periodic {
  double L, W;
  std::vector<double> goal_y;
  std::vector<int> dir;   // +1 / -1
};

void stepPeriodic(Model& m, std::vector<Pedestrian>& peds, const Periodic& pc) {
  const std::size_t n = peds.size();
  const double margin = 6.0;
  std::vector<Pedestrian> ext = peds;
  for (std::size_t i = 0; i < n; ++i) ext[i].goal = {peds[i].pos.x + 1000.0 * pc.dir[i], pc.goal_y[i]};
  for (std::size_t i = 0; i < n; ++i) {
    for (int s = -1; s <= 1; s += 2) {
      const double sx = s * pc.L;
      const double gx = peds[i].pos.x + sx;
      if (gx < -margin || gx > pc.L + margin) continue;
      if (s == 1 && peds[i].pos.x > margin) continue;
      if (s == -1 && peds[i].pos.x < pc.L - margin) continue;
      Pedestrian g = ext[i];
      g.pos.x = gx;
      if (ext.size() < 2 * n) ext.push_back(g);
    }
  }
  const std::size_t used = ext.size();
  while (ext.size() < 2 * n) {   // 固定サイズに埋める(遠方の凍結ダミー)
    Pedestrian d;
    d.id = 100000 + static_cast<uint32_t>(ext.size());
    d.pos = {1e4 + 10.0 * static_cast<double>(ext.size()), 100.0};
    d.home = d.pos; d.goal = d.pos; d.frozen = true;
    ext.push_back(d);
  }
  (void)used;
  m.step(ext);
  for (std::size_t i = 0; i < n; ++i) {
    peds[i].pos = ext[i].pos; peds[i].vel = ext[i].vel;
    while (peds[i].pos.x >= pc.L) peds[i].pos.x -= pc.L;
    while (peds[i].pos.x < 0.0) peds[i].pos.x += pc.L;
  }
}

double wrapDx(double dx, double L) {
  dx = std::fmod(dx, L);
  if (dx > 0.5 * L) dx -= L;
  if (dx < -0.5 * L) dx += L;
  return dx;
}

std::vector<Pedestrian> initPeriodic(std::mt19937& rng, int n, const Periodic& pc, const std::vector<int>& dir) {
  std::vector<Pedestrian> peds;
  const double ymax = 0.5 * pc.W - 0.4;
  for (int tries = 0; static_cast<int>(peds.size()) < n; ++tries) {
    if (tries > 200000) { std::fprintf(stderr, "init failed\n"); std::exit(1); }
    const Vec2 p{uni(rng, 0.0, pc.L), uni(rng, -ymax, ymax)};
    bool ok = true;
    for (const Pedestrian& q : peds) ok = ok && std::hypot(wrapDx(p.x - q.pos.x, pc.L), p.y - q.pos.y) > 0.45;
    if (!ok) continue;
    const int d = dir[peds.size()];
    Pedestrian a = mk(static_cast<uint32_t>(peds.size() + 1), p, {p.x + 100.0 * d, p.y}, walkSpeed(rng));
    peds.push_back(a);
  }
  return peds;
}

// ---- 2. 密度-速度(一方向流)
double densitySpeed(bool sfm2, int seed, double rho) {
  Periodic pc{20.0, 4.0, {}, {}};
  const int n = static_cast<int>(std::lround(rho * pc.L * pc.W));
  std::mt19937 rng(static_cast<unsigned>(seed));
  pc.dir.assign(n, 1);
  std::vector<Pedestrian> peds = initPeriodic(rng, n, pc, pc.dir);
  for (const Pedestrian& p : peds) pc.goal_y.push_back(p.pos.y);
  Model m(sfm2, longCorridor(0.5 * pc.W), static_cast<uint32_t>(seed));
  double sum = 0; long cnt = 0;
  const int steps = 2000;   // 40 s。後半 20 s を平均
  for (int k = 0; k < steps; ++k) {
    stepPeriodic(m, peds, pc);
    if (k >= steps / 2) { for (const Pedestrian& p : peds) { sum += p.vel.x; ++cnt; } }
  }
  return sum / static_cast<double>(cnt);
}

// ---- 4. 対面流(密度 1.0、左右 40 / 40)。車線の秩序(y を 0.5 m の帯に分けた純度の平均)と歩行者同士の接触回数
struct Counter { double order; double contacts; double speed; };
Counter counterFlow(bool sfm2, int seed) {
  Periodic pc{20.0, 4.0, {}, {}};
  const int n = 80;
  std::mt19937 rng(static_cast<unsigned>(seed));
  pc.dir.resize(n);
  for (int i = 0; i < n; ++i) pc.dir[i] = (i % 2 == 0) ? 1 : -1;
  std::vector<Pedestrian> peds = initPeriodic(rng, n, pc, pc.dir);
  for (const Pedestrian& p : peds) pc.goal_y.push_back(p.pos.y);
  Model m(sfm2, longCorridor(0.5 * pc.W), static_cast<uint32_t>(seed));
  std::vector<char> in_contact(static_cast<std::size_t>(n) * n, 0);
  double contacts = 0, order_sum = 0, speed_sum = 0; long order_cnt = 0;
  const int steps = 3000;   // 60 s
  for (int k = 0; k < steps; ++k) {
    stepPeriodic(m, peds, pc);
    for (int i = 0; i < n; ++i)
      for (int j = i + 1; j < n; ++j) {
        const double d = std::hypot(wrapDx(peds[i].pos.x - peds[j].pos.x, pc.L), peds[i].pos.y - peds[j].pos.y);
        char& c = in_contact[static_cast<std::size_t>(i) * n + j];
        if (d < 2 * kR) { if (!c) contacts += 1; c = 1; } else c = 0;
      }
    if (k >= steps * 2 / 3 && k % 25 == 0) {
      int right[8] = {0}, left[8] = {0};
      for (int i = 0; i < n; ++i) {
        const int b = std::min(7, std::max(0, static_cast<int>(std::floor((peds[i].pos.y + 2.0) / 0.5))));
        if (pc.dir[i] > 0) ++right[b]; else ++left[b];
      }
      int diff = 0;
      for (int b = 0; b < 8; ++b) diff += std::abs(right[b] - left[b]);
      order_sum += static_cast<double>(diff) / n;
      for (const Pedestrian& p : peds) speed_sum += std::fabs(p.vel.x);
      ++order_cnt;
    }
  }
  return {order_sum / order_cnt, contacts, speed_sum / (static_cast<double>(order_cnt) * n)};
}

// ---- 3 / 6. 正面 2 人。a の回避開始距離は「partner を横 10 m に離した同 seed の対照走行」との横ずれ 0.05 m で決める
struct HeadOn { double min_gap; double avoid_dist; };
HeadOn headOn(bool sfm2, int seed, int kind_a) {
  std::mt19937 rng(static_cast<unsigned>(seed));
  const double ya = uni(rng, -0.1, 0.1), yb = uni(rng, -0.1, 0.1);
  const double va = walkSpeed(rng), vb = walkSpeed(rng);
  auto run = [&](double yoff, std::vector<Vec2>* traj, std::vector<double>* dist) {
    Model m(sfm2, {}, static_cast<uint32_t>(seed));
    std::vector<Pedestrian> peds{mk(1, {-6, ya}, {8, ya}, va), mk(2, {6, yb + yoff}, {-8, yb + yoff}, vb)};
    peds[0].kind = kind_a;
    if (kind_a == 2) peds[0].attention = 1.0;
    double min_d = 1e9;
    for (int k = 0; k < 700; ++k) {   // 14 s
      m.step(peds);
      const double d = std::hypot(peds[0].pos.x - peds[1].pos.x, peds[0].pos.y - peds[1].pos.y);
      min_d = std::min(min_d, d);
      if (traj) traj->push_back(peds[0].pos);
      if (dist) dist->push_back(d);
    }
    return min_d;
  };
  std::vector<Vec2> t_real, t_ctrl;
  std::vector<double> d_real;
  const double min_d = run(0.0, &t_real, &d_real);
  run(10.0, &t_ctrl, nullptr);
  double avoid = 0.0;
  for (std::size_t k = 0; k < t_real.size(); ++k) {
    if (std::fabs(t_real[k].y - t_ctrl[k].y) > 0.05) { avoid = d_real[k]; break; }
  }
  return {min_d - 2 * kR, avoid};
}

// ---- 5. 家族: 3 人が並んで 30 s 歩く(対向の単独歩行者 8 人あり、幅 4 m の通路)。群内の平均ペア距離
struct Family { double mean_dist; double max_dist; };
Family familyWalk(bool sfm2, int seed) {
  std::mt19937 rng(static_cast<unsigned>(seed));
  Model m(sfm2, longCorridor(2.0), static_cast<uint32_t>(seed));
  std::vector<Pedestrian> peds;
  const double ys[3] = {-0.7, 0.0, 0.7};
  for (int i = 0; i < 3; ++i) {
    Pedestrian f = mk(static_cast<uint32_t>(i + 1), {0.0, ys[i]}, {200.0, ys[i]}, 1.2);
    f.kind = 1; f.group_id = 0;
    peds.push_back(f);
  }
  for (int i = 0; i < 8; ++i) {
    const double x = uni(rng, 10.0, 50.0), y = uni(rng, -1.5, 1.5);
    peds.push_back(mk(static_cast<uint32_t>(4 + i), {x, y}, {-200.0, y}, walkSpeed(rng)));
  }
  double sum = 0, mx = 0; long cnt = 0;
  for (int k = 0; k < 1500; ++k) {   // 30 s
    m.step(peds);
    double d = 0;
    for (int i = 0; i < 3; ++i) for (int j = i + 1; j < 3; ++j)
      d += std::hypot(peds[i].pos.x - peds[j].pos.x, peds[i].pos.y - peds[j].pos.y);
    d /= 3.0;
    sum += d; mx = std::max(mx, d); ++cnt;
  }
  return {sum / static_cast<double>(cnt), mx};
}


// ---- H0(接触直前の止まり方の規則): sfm2 が静止した外形に対して不自然に当たらないか。人 1 人、(-8,0) -> (8,0)、壁なし、20 s。
// (a) 静止ロボット(RobotBody、vel 0)を (0, y) / (b) 同じ位置に固定の歩行者(位置と速度 0 を毎ステップ戻す、ロボットは nullptr)
// (c) 対向のロボット 0.8 m/s。横ずれ = 等速の最接近ですきま 0.15 m、最接近の 1 s 前から 1.0 m/s^2 で減速して停止(記録だけ)。
// 接触 = 外形間すきま < 0 が 1 回でもあるシード。yoff は配置の検査用(ロボット / 固定歩行者を横にずらす。0 が本番)
constexpr int kH0Seed0 = 1021, kH0N = 100;
constexpr double kH0Steps = 1000;   // 20 s

struct H0Out { int contact; double min_gap; };

H0Out h0Static(int seed, double y, bool as_robot, double yoff) {
  std::mt19937 rng(static_cast<unsigned>(seed));
  const double v0 = walkSpeed(rng);
  PedModel m(PedParams{}, {}, static_cast<uint32_t>(seed) + 4242u);
  std::vector<Pedestrian> peds{mk(1, {-8, 0}, {8, 0}, v0)};
  const Vec2 fix{0.0, y + yoff};
  if (!as_robot) {
    Pedestrian f;
    f.id = 2; f.pos = fix; f.home = fix; f.goal = fix; f.v0 = 0.0; f.radius = kR;
    peds.push_back(f);
  }
  const RobotBody body{fix, {0, 0}, kR};
  double mg = 1e9;
  for (int k = 0; k < kH0Steps; ++k) {
    m.step(peds, as_robot ? &body : nullptr, kDt);
    if (!as_robot) { peds[1].pos = fix; peds[1].vel = {0, 0}; }
    mg = std::min(mg, std::hypot(peds[0].pos.x - fix.x, peds[0].pos.y - fix.y) - 2 * kR);
  }
  return {mg < 0.0 ? 1 : 0, mg};
}

H0Out h0Oncoming(int seed, double yoff) {
  std::mt19937 rng(static_cast<unsigned>(seed));
  const double v0 = walkSpeed(rng);
  const double vr = 0.8, lateral = 2 * kR + 0.15;   // 中心間の最接近 0.75 m = 外形間 0.15 m
  const double t_star = 16.0 / (v0 + vr);            // 等速の最接近の時刻(人 x = -8 + v0 t、ロボット x = 8 - vr t)
  PedModel m(PedParams{}, {}, static_cast<uint32_t>(seed) + 4242u);
  std::vector<Pedestrian> peds{mk(1, {-8, 0}, {8, 0}, v0)};
  Vec2 rp{8.0, lateral + yoff};
  double rv = vr, mg = 1e9;
  for (int k = 0; k < kH0Steps; ++k) {
    const double t = k * kDt;
    if (t >= t_star - 1.0) rv = std::max(0.0, rv - 1.0 * kDt);
    const RobotBody body{rp, {-rv, 0.0}, kR};
    m.step(peds, &body, kDt);
    rp.x -= rv * kDt;
    mg = std::min(mg, std::hypot(peds[0].pos.x - rp.x, peds[0].pos.y - rp.y) - 2 * kR);
  }
  return {mg < 0.0 ? 1 : 0, mg};
}

int runH0(int seed0) {
  std::printf("ped_probe --h0: sfm2 のみ、シード %d-%d (%d)、dt %.2f s、人 1 人 (-8,0)->(8,0)、20 s、半径 %.1f m\n\n", seed0,
              seed0 + kH0N - 1, kH0N, kDt, kR);
  const double ys[3] = {0.0, 0.2, 0.4};
  bool fix = false;
  std::printf("| y [m] | (a) 静止ロボット 接触 | (a) 率 | (b) 固定歩行者 接触 | (b) 率 | 対差 a-b の平均 | SE | (a) 率 > (b) 率 + 2SE | (a) 接触 >= 1 |\n|---|---|---|---|---|---|---|---|---|\n");
  for (double y : ys) {
    std::vector<double> d; int ca = 0, cb = 0;
    for (int seed = seed0; seed < seed0 + kH0N; ++seed) {
      const int a = h0Static(seed, y, true, 0.0).contact, b = h0Static(seed, y, false, 0.0).contact;
      ca += a; cb += b; d.push_back(a - b);
    }
    const double md = mean(d);
    double ss = 0; for (double x : d) ss += (x - md) * (x - md);
    const double se = d.size() > 1 ? std::sqrt(ss / static_cast<double>(d.size() - 1)) / std::sqrt(static_cast<double>(kH0N)) : 0.0;
    const double ra = ca / static_cast<double>(kH0N), rb = cb / static_cast<double>(kH0N);
    const bool c1 = ra > rb + 2 * se, c2 = ca >= 1;
    fix = fix || c1 || c2;
    std::printf("| %.1f | %d | %.2f | %d | %.2f | %.3f | %.4f | %s | %s |\n", y, ca, ra, cb, rb, md, se, c1 ? "yes" : "no", c2 ? "yes" : "no");
  }
  {
    int cc = 0; std::vector<double> g;
    for (int seed = seed0; seed < seed0 + kH0N; ++seed) { const H0Out o = h0Oncoming(seed, 0.0); cc += o.contact; g.push_back(o.min_gap); }
    std::printf("\n(c) 対向ロボット 0.8 m/s(横ずれ %.2f m、最接近の 1 s 前から 1.0 m/s^2 で停止。記録だけ): 接触 %d / %d(率 %.2f)、最小すきま 平均 %.3f m、最小 %.3f m\n",
                2 * kR + 0.15, cc, kH0N, cc / static_cast<double>(kH0N), mean(g), vmin(g));
  }
  {
    // 配置の検査: ロボット / 固定歩行者を 10 m 横に離す。接触は 0 のはず(結果には数えない)
    int ca = 0, cb = 0, cc = 0;
    for (int seed = seed0; seed < seed0 + kH0N; ++seed)
      for (double y : ys) {
        ca += h0Static(seed, y, true, 10.0).contact; cb += h0Static(seed, y, false, 10.0).contact;
      }
    for (int seed = seed0; seed < seed0 + kH0N; ++seed) cc += h0Oncoming(seed, 10.0).contact;
    std::printf("\n配置の検査(横 10 m 離した対照。結果に数えない): (a) 接触 %d、(b) 接触 %d、(c) 接触 %d(全て 0 のはず)%s\n", ca, cb, cc,
                (ca + cb + cc == 0) ? "" : "  <-- 配置が不正");
  }
  std::printf("\nH0 判定(直す条件: いずれかの y で (a) 接触 >= 1、または (a) 率 > (b) 率 + 2SE。(c) は理由にしない): %s\n",
              fix ? "該当(別 issue。この issue では sfm2 を変えない)" : "該当なし");
  return 0;
}

struct Row { std::string name, band; double legacy, sfm2; bool pass; };

void pr(const char* item, const char* band, const char* fmt_l, double l, double s, const char* verdict) {
  char lb[64], sb[64];
  std::snprintf(lb, sizeof lb, fmt_l, l);
  std::snprintf(sb, sizeof sb, fmt_l, s);
  std::printf("| %s | %s | %s | %s | %s |\n", item, band, lb, sb, verdict);
}
const char* ok(bool b) { return b ? "PASS" : "FAIL"; }

}  // namespace

int main(int argc, char** argv) {
  // --h0 [--h0-seed0 N]: H0 だけを実行する(既定の出力は変えない)。--h0-seed0 は動作確認用(本番は既定の 1021)
  bool h0 = false; int h0_seed0 = kH0Seed0;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--h0") h0 = true;
    else if (a == "--h0-seed0" && i + 1 < argc) h0_seed0 = std::atoi(argv[++i]);
    else { std::fprintf(stderr, "usage: ped_probe [--h0 [--h0-seed0 N]]\n"); return 2; }
  }
  if (h0) return runH0(h0_seed0);
  std::printf("ped_probe: seeds %d-%d (%d), dt %.2f s, radius %.1f m. 合否は sfm2 について(legacy は参考)\n\n",
              kSeed0, kSeed1, kSeed1 - kSeed0 + 1, kDt, kR);
  std::printf("| 項目 | 帯 | legacy | sfm2 | sfm2 合否 |\n|---|---|---|---|---|\n");
  int fails = 0;
  auto verdict = [&](bool b) { if (!b) ++fails; return ok(b); };
  auto sweep = [&](const std::function<double(bool, int)>& f, double& l, double& s) {
    std::vector<double> a, b;
    for (int seed = kSeed0; seed <= kSeed1; ++seed) { a.push_back(f(false, seed)); b.push_back(f(true, seed)); }
    l = mean(a); s = mean(b);
  };
  auto sweepV = [&](const std::function<double(bool, int)>& f, std::vector<double>& a, std::vector<double>& b) {
    for (int seed = kSeed0; seed <= kSeed1; ++seed) { a.push_back(f(false, seed)); b.push_back(f(true, seed)); }
  };

  double l, s;
  // 1 自由流
  sweep([](bool m, int seed) { return freeFlowRatio(m, seed, 0); }, l, s);
  pr("自由流 速度/v0", "0.95-1.05", "%.3f", l, s, verdict(std::fabs(s - 1.0) <= 0.05));

  // 2 密度-速度
  const double rho[3] = {0.5, 1.0, 1.5}, weid[3] = {1.30, 1.06, 0.80};
  double ds[3], dl[3];
  for (int i = 0; i < 3; ++i) {
    sweep([&](bool m, int seed) { return densitySpeed(m, seed, rho[i]); }, dl[i], ds[i]);
    char item[64], band[64];
    std::snprintf(item, sizeof item, "密度 %.1f 人/m2 速度 [m/s]", rho[i]);
    std::snprintf(band, sizeof band, "%.3f-%.3f", weid[i] * 0.75, weid[i] * 1.25);
    pr(item, band, "%.3f", dl[i], ds[i], verdict(ds[i] >= weid[i] * 0.75 && ds[i] <= weid[i] * 1.25));
  }
  pr("密度-速度 単調減少", "v(0.5)>v(1.0)>v(1.5)", "%.0f", (dl[0] > dl[1] && dl[1] > dl[2]) ? 1.0 : 0.0,
     (ds[0] > ds[1] && ds[1] > ds[2]) ? 1.0 : 0.0, verdict(ds[0] > ds[1] && ds[1] > ds[2]));

  // 3 正面 2 人
  {
    std::vector<double> gl, gs, al, as;
    for (int seed = kSeed0; seed <= kSeed1; ++seed) {
      const HeadOn a = headOn(false, seed, 0), b = headOn(true, seed, 0);
      gl.push_back(a.min_gap); gs.push_back(b.min_gap); al.push_back(a.avoid_dist); as.push_back(b.avoid_dist);
    }
    pr("正面 min gap 最小 [m]", "> 0 (全 seed)", "%.3f", vmin(gl), vmin(gs), verdict(vmin(gs) > 0.0));
    int cl = 0, cs = 0;
    for (double x : gl) cl += x <= 0.0;
    for (double x : gs) cs += x <= 0.0;
    pr("正面 接触した seed 数", "0", "%.0f", cl, cs, "-");
    pr("正面 回避開始距離 平均 [m]", ">= 2", "%.2f", mean(al), mean(as), verdict(mean(as) >= 2.0));
    pr("正面 回避開始距離 最小 [m]", "参考", "%.2f", vmin(al), vmin(as), "-");
  }

  // 4 対面流
  {
    std::vector<double> ol, os, cl, cs;
    for (int seed = kSeed0; seed <= kSeed1; ++seed) {
      const Counter a = counterFlow(false, seed), b = counterFlow(true, seed);
      ol.push_back(a.order); os.push_back(b.order); cl.push_back(a.contacts); cs.push_back(b.contacts);
    }
    pr("対面流 車線の秩序(0-1)", "sfm2 > legacy", "%.3f", mean(ol), mean(os), verdict(mean(os) > mean(ol)));
    pr("対面流 人同士の接触回数/60 s", "sfm2 <= legacy", "%.1f", mean(cl), mean(cs), verdict(mean(cs) <= mean(cl)));
  }

  // 5 家族
  {
    std::vector<double> ml, ms, xl, xs;
    for (int seed = kSeed0; seed <= kSeed1; ++seed) {
      const Family a = familyWalk(false, seed), b = familyWalk(true, seed);
      ml.push_back(a.mean_dist); ms.push_back(b.mean_dist); xl.push_back(a.max_dist); xs.push_back(b.max_dist);
    }
    pr("家族 群内平均距離 [m]", "<= 1.5", "%.2f", mean(ml), mean(ms), verdict(mean(ms) <= 1.5));
    pr("家族 群内平均距離の最大 [m]", "参考", "%.2f", vmax(xl), vmax(xs), "-");
  }

  // 6 歩きスマホ(legacy は kind を読まないので通常と同じ値になる)
  {
    double nl, ns, pl, ps;
    sweep([](bool m, int seed) { return freeFlowRatio(m, seed, 0); }, nl, ns);
    sweep([](bool m, int seed) { return freeFlowRatio(m, seed, 2); }, pl, ps);
    pr("スマホ 自由流 速度 [m/s](通常)", "-", "%.3f", nl * 1.3, ns * 1.3, "-");
    pr("スマホ 自由流 速度 [m/s](スマホ)", "< 通常", "%.3f", pl * 1.3, ps * 1.3, verdict(ps < ns));
    std::vector<double> an, ap, bn, bp;
    sweepV([](bool m, int seed) { return headOn(m, seed, 0).avoid_dist; }, an, bn);
    sweepV([](bool m, int seed) { return headOn(m, seed, 2).avoid_dist; }, ap, bp);
    pr("スマホ 回避開始距離 [m](通常)", "-", "%.2f", mean(an), mean(bn), "-");
    pr("スマホ 回避開始距離 [m](スマホ)", "< 通常(遅い)", "%.2f", mean(ap), mean(bp), verdict(mean(bp) < mean(bn)));
  }
  std::printf("\nsfm2 の帯外: %d 項目\n", fails);
  return 0;
}
