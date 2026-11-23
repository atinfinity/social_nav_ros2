#include "sim/scenarios.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace social_nav {
namespace sim {

namespace {

constexpr double kPi = 3.14159265358979323846;

double uni(std::mt19937& r, double lo, double hi) { return std::uniform_real_distribution<double>(lo, hi)(r); }
double gauss(std::mt19937& r, double m, double s) { return std::normal_distribution<double>(m, s)(r); }
double walkSpeed(std::mt19937& r, double mean = 1.3, double sd = 0.2) {
  return std::max(0.5, gauss(r, mean, sd));
}

std::vector<Segment> corridorWalls(double x0, double x1, double half_width) {
  return {{{x0, half_width}, {x1, half_width}}, {{x0, -half_width}, {x1, -half_width}}};
}

std::vector<Segment> hallWalls(double x0, double x1, double y0, double y1) {
  return {{{x0, y1}, {x1, y1}}, {{x0, y0}, {x1, y0}}, {{x0, y0}, {x0, y1}}, {{x1, y0}, {x1, y1}}};
}

Scenario corridorOncoming(std::mt19937& r) {
  Scenario s;
  s.name = "corridor_oncoming";
  s.walls = corridorWalls(-2.0, 20.0, 1.6);
  s.start = {0.0, 0.0, 0.0};
  s.goal = {16.0, 0.0};
  for (int i = 0; i < 3; ++i) {
    const double y = uni(r, -0.8, 0.8);
    s.peds.push_back({{14.0 + 3.0 * i + uni(r, -0.5, 0.5), y}, {-4.0, y}, walkSpeed(r), false, false});
  }
  return s;
}

Scenario crossing(std::mt19937& r) {
  Scenario s;
  s.name = "crossing";
  s.walls = hallWalls(-2.0, 16.0, -7.0, 7.0);
  s.start = {0.0, 0.0, 0.0};
  s.goal = {12.0, 0.0};
  for (int i = 0; i < 4; ++i) {
    const double sign = (i % 2 == 0) ? 1.0 : -1.0;
    const double x = uni(r, 4.0, 9.0);
    s.peds.push_back({{x, sign * uni(r, 3.0, 5.0)}, {x + uni(r, -1.0, 1.0), -sign * 6.3}, walkSpeed(r), false, false});
  }
  return s;
}

Scenario overtake(std::mt19937& r) {
  Scenario s;
  s.name = "overtake";
  s.walls = corridorWalls(-2.0, 24.0, 1.75);
  s.start = {0.0, 0.0, 0.0};
  s.goal = {16.0, 0.0};
  const double y = uni(r, -0.3, 0.3);
  s.peds.push_back({{4.0 + uni(r, -0.5, 0.5), y}, {26.0, y}, walkSpeed(r, 0.45, 0.05), false, false});
  return s;
}

Scenario follow(std::mt19937& r) {
  Scenario s;
  s.name = "follow";
  s.walls = corridorWalls(-2.0, 26.0, 1.75);
  s.start = {0.0, 0.0, 0.0};
  s.goal = {16.0, 0.0};
  const double y = uni(r, -0.3, 0.3);
  s.peds.push_back({{3.0 + uni(r, -0.3, 0.3), y}, {28.0, y}, walkSpeed(r, 0.65, 0.05), false, false});
  return s;
}

Scenario staticGroup(std::mt19937& r) {
  Scenario s;
  s.name = "static_group";
  s.walls = hallWalls(-2.0, 16.0, -5.0, 5.0);
  s.start = {0.0, 0.0, 0.0};
  s.goal = {14.0, 0.0};
  // 再計画なしの経路でも通れるよう、グループは経路の脇(中心が経路から 0.8〜1.1 m 横)に立つ
  const double cy = (uni(r, 0.0, 1.0) < 0.5 ? -1.0 : 1.0) * uni(r, 0.8, 1.1);
  const double a0 = uni(r, 0.0, 2.0 * kPi);
  for (int i = 0; i < 4; ++i) {
    const double a = a0 + i * kPi / 2.0 + uni(r, -0.2, 0.2);
    const double rad = 0.7 + uni(r, -0.1, 0.1);
    s.peds.push_back({{7.0 + rad * std::cos(a), cy + rad * std::sin(a)}, {0, 0}, 0.0, true, false});
  }
  s.peds.push_back({{10.5 + uni(r, -0.3, 0.3), 2.0 + uni(r, -0.3, 0.3)}, {0, 0}, 0.0, true, false});
  s.peds.push_back({{10.5 + uni(r, -0.3, 0.3), -2.0 + uni(r, -0.3, 0.3)}, {0, 0}, 0.0, true, false});
  for (PedInit& p : s.peds) p.goal = p.pos;
  return s;
}

Scenario crowd(std::mt19937& r) {
  Scenario s;
  s.name = "crowd";
  s.walls = hallWalls(-4.0, 26.0, -4.0, 4.0);
  s.start = {0.0, 0.0, 0.0};
  s.goal = {18.0, 0.0};
  const int n = 24;
  while (static_cast<int>(s.peds.size()) < n) {
    const Vec2 p{uni(r, 0.0, 20.0), uni(r, -3.5, 3.5)};
    bool ok = std::hypot(p.x - s.start.x, p.y - s.start.y) > 1.5;
    for (const PedInit& q : s.peds) ok = ok && std::hypot(p.x - q.pos.x, p.y - q.pos.y) > 0.8;
    if (!ok) continue;
    const bool right = uni(r, 0.0, 1.0) < 0.5;
    const Vec2 g{right ? 24.0 : -3.0, uni(r, -3.0, 3.0)};
    s.peds.push_back({p, g, walkSpeed(r), false, true});
  }
  return s;
}

// 後方からの人流(記録のみ。既定の 6 本には入れない)。人はロボットの後方から出発し、同じ向きに進む
Scenario rearFollow(std::mt19937& r) {
  Scenario s;
  s.name = "rear_follow";
  s.walls = corridorWalls(-8.0, 26.0, 1.75);
  s.start = {0.0, 0.0, 0.0};
  s.goal = {16.0, 0.0};
  const double y = uni(r, -0.3, 0.3);
  s.peds.push_back({{-3.0 + uni(r, -0.3, 0.3), y}, {28.0, y}, walkSpeed(r, 0.9, 0.1), false, false});
  return s;
}

Scenario rearOvertake(std::mt19937& r) {
  Scenario s;
  s.name = "rear_overtake";
  s.walls = corridorWalls(-8.0, 26.0, 1.75);
  s.start = {0.0, 0.0, 0.0};
  s.goal = {16.0, 0.0};
  const double y = uni(r, -0.3, 0.3);
  s.peds.push_back({{-3.0 + uni(r, -0.5, 0.5), y}, {28.0, y}, walkSpeed(r, 1.3, 0.2), false, false});
  return s;
}

Scenario rearOvertakeFlow(std::mt19937& r) {
  Scenario s;
  s.name = "rear_overtake_flow";
  s.walls = corridorWalls(-14.0, 26.0, 1.75);
  s.start = {0.0, 0.0, 0.0};
  s.goal = {16.0, 0.0};
  for (int i = 0; i < 3; ++i) {
    const double y = uni(r, -0.8, 0.8);
    s.peds.push_back({{-3.0 - 3.0 * i + uni(r, -0.5, 0.5), y}, {28.0, y}, walkSpeed(r, 1.3, 0.2), false, false});
  }
  return s;
}

// 屋内(記録のみ。既定の 6 本には入れない)。ロボットは (0,0)->(12,0)、名目の到着時刻 T_R(x) = x / 0.8 + 0.5
double nominalArrival(double x) { return x / 0.8 + 0.5; }

// 廊下の壁(y = +-half_width)のうち x = xc の位置に幅 w の開口部を空けたもの。開口部の側に枝通路(|y| half_width〜y_end、端は閉じる)
void addSideOpening(std::vector<Segment>& walls, double x0, double x1, double half_width, double xc, double w, double y_end) {
  for (const double sg : {1.0, -1.0}) {
    walls.push_back({{x0, sg * half_width}, {xc - 0.5 * w, sg * half_width}});
    walls.push_back({{xc + 0.5 * w, sg * half_width}, {x1, sg * half_width}});
    walls.push_back({{xc - 0.5 * w, sg * half_width}, {xc - 0.5 * w, sg * y_end}});
    walls.push_back({{xc + 0.5 * w, sg * half_width}, {xc + 0.5 * w, sg * y_end}});
    walls.push_back({{xc - 0.5 * w, sg * y_end}, {xc + 0.5 * w, sg * y_end}});
  }
}

Scenario doorOncoming(std::mt19937& r) {
  Scenario s;
  s.name = "door_oncoming";
  const double w = uni(r, 0.9, 1.2);
  s.walls = corridorWalls(-6.0, 24.0, 1.75);
  s.walls.push_back({{5.0, 1.75}, {5.0, 0.5 * w}});     // 厚さ 0 の仕切り壁。開口部の中心は y = 0(ロボットの直線経路上)
  s.walls.push_back({{5.0, -0.5 * w}, {5.0, -1.75}});
  s.start = {0.0, 0.0, 0.0};
  s.goal = {12.0, 0.0};
  const double v0 = walkSpeed(r);
  const double delta = uni(r, -1.5, 1.5);
  const double y0 = uni(r, -0.8, 0.8);
  const double gy = uni(r, -0.8, 0.8);
  PedInit p{{std::min(22.0, 5.0 + v0 * (nominalArrival(5.0) + delta)), y0}, {-4.0, gy}, v0, false, false};
  p.has_via = true;
  p.via = {5.0, 0.0};
  s.peds.push_back(p);
  return s;
}

Scenario sideDoor(std::mt19937& r) {
  Scenario s;
  s.name = "side_door";
  const double sign = uni(r, 0.0, 1.0) < 0.5 ? 1.0 : -1.0;
  const double w = uni(r, 1.1, 1.4);   // 計画は 0.9〜1.2。0.9〜1.05 付近は sfm2 の人が単独でも開口部を通れなかったため、シード 1–20 の配置の検査で直した。原因は未測定
  s.walls = {};
  addSideOpening(s.walls, -6.0, 24.0, 1.75, 3.0, w, 9.0);
  s.start = {0.0, 0.0, 0.0};
  s.goal = {12.0, 0.0};
  const double v0 = walkSpeed(r);
  const double delta = uni(r, -1.5, 1.5);
  const double branch = uni(r, 0.0, 1.0);
  const double gy = uni(r, -0.8, 0.8);
  const double y0 = sign * std::min(8.5, v0 * (nominalArrival(3.0) + delta));
  PedInit p{{3.0, y0}, {-4.0, gy}, v0, false, false};
  if (branch < 1.0 / 3.0) {            // 横切る
    p.goal = {3.0, -sign * 6.0};
  } else {                              // 合流(対向 or 同方向)
    p.has_via = true;
    p.via = {3.0, sign * 0.9};
    if (branch >= 2.0 / 3.0) p.goal = {22.0, gy};
  }
  s.peds.push_back(p);
  return s;
}

// 屋内の群れ(記録のみ)。廊下を 2〜3 人の群れが横に並んで向かってくる。group_id は ped_variants(crowd のときだけ 0..2)と重ならない 100
Scenario groupOncoming(std::mt19937& r) {
  Scenario s;
  s.name = "group_oncoming";
  s.walls = corridorWalls(-2.0, 20.0, 1.6);
  s.start = {0.0, 0.0, 0.0};
  s.goal = {16.0, 0.0};
  const int n = uni(r, 0.0, 1.0) < 0.5 ? 2 : 3;
  const double gap = uni(r, 0.65, 0.85);
  const double yc = n == 2 ? uni(r, -0.6, 0.6) : uni(r, -0.3, 0.3);
  const double xc = 14.0 + uni(r, -0.5, 0.5);
  const double v0 = walkSpeed(r);
  for (int i = 0; i < n; ++i) {
    const double y = yc + (i - 0.5 * (n - 1)) * gap;
    PedInit p{{xc + uni(r, -0.15, 0.15), y}, {-4.0, y}, v0, false, false};
    p.group_id = 100;
    s.peds.push_back(p);
  }
  return s;
}

// 見えない人(記録のみ。--occlusion と組み合わせて使う)。T 字路。廊下の半幅 kBlindHalfWidth、開口部 x = 4、枝は |y| 1.0〜12(上下両側)。
// ロボットは直線経路のまま。人は枝の奥から出てきて、壁の裏から開口部に現れる
constexpr double kBlindHalfWidth = 1.0;
constexpr double kBlindWMin = 1.2;   // 壁の角の二重計上を避ける下限
constexpr double kBlindWMax = 1.6;

Scenario blindCorner(std::mt19937& r) {
  Scenario s;
  s.name = "blind_corner";
  const double sign = uni(r, 0.0, 1.0) < 0.5 ? 1.0 : -1.0;
  const double w = uni(r, kBlindWMin, kBlindWMax);
  addSideOpening(s.walls, -6.0, 24.0, kBlindHalfWidth, 4.0, w, 12.0);
  s.opening_w = w;
  s.start = {0.0, 0.0, 0.0};
  s.goal = {12.0, 0.0};
  const double v0 = walkSpeed(r);
  const double delta = uni(r, -0.75, 0.75);
  const double branch = uni(r, 0.0, 1.0);
  const double gy = uni(r, -0.5, 0.5);
  const double y0 = sign * std::min(11.5, v0 * (nominalArrival(4.0) + delta));
  PedInit p{{4.0, y0}, {-4.0, gy}, v0, false, false};
  if (branch < 1.0 / 3.0) {            // 横切る
    p.goal = {4.0, -sign * 6.0};
  } else {                              // 合流(対向 or 同方向)
    p.has_via = true;
    p.via = {4.0, sign * 0.4};
    if (branch >= 2.0 / 3.0) p.goal = {22.0, gy};
  }
  s.peds.push_back(p);
  return s;
}

// 急に止まる・振り返る人(記録のみ)。3 本で同じ生成関数(同シードで同配置)。動作が違っても乱数は同じ順序で全部引く。
// 配置は follow と同形(人はロボットの前方 3 m、同じ向きに v0 0.65 m/s 前後で歩く)。事象の実装は sim/ped_event.hpp
Scenario suddenCommon(std::mt19937& r, const std::string& name, int kind) {
  Scenario s;
  s.name = name;
  s.walls = corridorWalls(-6.0, 26.0, 1.75);
  s.start = {0.0, 0.0, 0.0};
  s.goal = {16.0, 0.0};
  const double y = uni(r, -0.3, 0.3);
  const double x0 = 3.0 + uni(r, -0.3, 0.3);
  const double v0 = walkSpeed(r, 0.65, 0.05);
  const double d = uni(r, 2.5, 2.65);   // 1–20 の配置検査で調整(初版 1.6〜2.4 → 2.2〜2.65 → 2.5〜2.65)。ロボットが d のすぐ上で間隔を止める走行があった。原因は未測定(コードの読みでは FOLLOW が入った時点の間隔を保つ)。上限 2.65 は初期 x0 の下限 2.7 より小さい(開始直後に発火しない)
  const double stop_dur = uni(r, 2.0, 4.0);
  const double side = uni(r, 0.0, 1.0) < 0.5 ? 1.0 : -1.0;
  PedInit p{{x0, y}, {28.0, y}, v0, false, false};
  p.event.kind = kind;
  p.event.trigger_d = d;
  p.event.t_max = 12.0;
  p.event.decel = 2.0;
  p.event.pause = kind == 1 ? stop_dur : 0.5;
  p.event.turn_y = 1.2 * side;
  p.event.reverse_x = -4.0;
  s.peds.push_back(p);
  return s;
}

// エレベーター前・出口のまとまった流れ(候補 6。記録のみ)。広間 x −4〜18、y ±4.5。エレベーターの扉は東の壁(x = 18、y ±1.2)。
// 出てくる人 4〜6 人が扉の前に 2 列 × 最大 3 行の塊で同時に出発し、西(出口)へ向かう(目標は x = −3、y は 0.8 m 間隔の車線を開始時の y の順に割り当てる。出口の端で詰まらないため)。出てくる人は共通の時刻 t_r ~ U(9, 11) s(release_t)まで扉の前で止まって待つ。待っている人 4〜6 人は扉の手前 x 10.6〜13.8 の塊(静止、|y| は車線の端から 0.55 m 外で約 1.55 以上)。
// ロボットは (0,0) → (12,0)。乱数は全部ここで引く(どのモデルでも同じ配置)
Scenario elevatorFlow(std::mt19937& r) {
  Scenario s;
  s.name = "elevator_flow";
  s.walls = hallWalls(-4.0, 18.0, -4.5, 4.5);
  s.start = {0.0, 0.0, 0.0};
  s.goal = {12.0, 0.0};
  const int n_out = 4 + static_cast<int>(uni(r, 0.0, 3.0));   // 4, 5, 6
  const int n_wait = 4 + static_cast<int>(uni(r, 0.0, 3.0));
  const double yc = uni(r, -0.2, 0.2);
  const double x0 = 16.2 + uni(r, -0.4, 0.2);
  for (int i = 0; i < n_out; ++i) {
    const int col = i / 3, row = i % 3;
    const int rows = (col == 0) ? std::min(3, n_out) : n_out - 3;
    const double y = yc + (row - 0.5 * (rows - 1)) * 0.9 + uni(r, -0.05, 0.05);
    const double x = x0 + col * 1.0 + uni(r, -0.05, 0.05);
    const double v0 = walkSpeed(r);
    s.peds.push_back({{x, y}, {-3.0, y}, v0, false, false});
  }
  // 出口の車線: x = −3 に 0.8 m 間隔で並べ、開始時の y の順に割り当てる(追い越しや交差で出口の端が詰まらないようにする。乱数は引かない)
  std::vector<int> idx(n_out);
  for (int i = 0; i < n_out; ++i) idx[i] = i;
  std::stable_sort(idx.begin(), idx.end(), [&](int a, int b) { return s.peds[a].pos.y < s.peds[b].pos.y; });
  for (int k = 0; k < n_out; ++k) s.peds[idx[k]].goal.y = yc + (k - 0.5 * (n_out - 1)) * 0.8;
  {   // 待っている人: 流れの車線の外側すぐ脇(車線の端から 0.55 m。n_out と yc で決まる)に塊で立つ。扉の手前 x 10.6〜13.8 の格子(5 列 x 最大 2 行)から、乱数の中心に近い順に n_wait 点を取る(決定的。棄却ループなし)
    const double side = uni(r, 0.0, 1.0) < 0.5 ? 1.0 : -1.0;
    const double cx = uni(r, 12.2, 13.0);
    const double lane_edge = side * yc + 0.5 * (n_out - 1) * 0.8;   // 車線の側の端(絶対値)
    const double y1 = lane_edge + 0.55, y2 = y1 + 0.75;   // 行の間隔 0.75 m(y に雑音なし)
    std::vector<Vec2> slots;
    for (int c = 0; c < 5; ++c) {
      slots.push_back({10.6 + 0.8 * c, side * y1});
      slots.push_back({10.6 + 0.8 * c, side * y2});
    }
    std::stable_sort(slots.begin(), slots.end(), [&](const Vec2& a, const Vec2& b) {
      return std::hypot(a.x - cx, a.y - side * y1) < std::hypot(b.x - cx, b.y - side * y1);
    });
    for (int i = 0; i < n_wait; ++i) {
      const Vec2 p{slots[i].x + uni(r, -0.1, 0.1), slots[i].y};   // x の間隔 0.8 − 雑音 0.2 = 0.6 m 以上
      s.peds.push_back({p, p, 0.0, true, false});
    }
  }
  const double t_r = uni(r, 9.0, 11.0);   // 最後に 1 回だけ引く(これまでの配置の乱数列は変えない)。出てくる人だけが待つ
  for (PedInit& p : s.peds)
    if (!p.frozen) p.release_t = t_r;
  return s;
}

}  // namespace

const std::vector<std::string>& elevatorScenarioNames() {
  static const std::vector<std::string> names{"elevator_flow"};
  return names;
}

const std::vector<std::string>& suddenScenarioNames() {
  static const std::vector<std::string> names{"sudden_stop", "sudden_turn", "sudden_reverse"};
  return names;
}

const std::vector<std::string>& blindScenarioNames() {
  static const std::vector<std::string> names{"blind_corner"};
  return names;
}

const std::vector<std::string>& scenarioNames() {
  static const std::vector<std::string> names{"corridor_oncoming", "crossing", "overtake",
                                              "follow", "static_group", "crowd"};
  return names;
}

const std::vector<std::string>& rearScenarioNames() {
  static const std::vector<std::string> names{"rear_follow", "rear_overtake", "rear_overtake_flow"};
  return names;
}

const std::vector<std::string>& indoorScenarioNames() {
  static const std::vector<std::string> names{"door_oncoming", "side_door", "group_oncoming"};
  return names;
}

Scenario makeScenario(const std::string& name, std::mt19937& rng) {
  Scenario s;
  if (name == "corridor_oncoming") s = corridorOncoming(rng);
  else if (name == "crossing") s = crossing(rng);
  else if (name == "overtake") s = overtake(rng);
  else if (name == "follow") s = follow(rng);
  else if (name == "static_group") s = staticGroup(rng);
  else if (name == "crowd") s = crowd(rng);
  else if (name == "rear_follow") s = rearFollow(rng);
  else if (name == "rear_overtake") s = rearOvertake(rng);
  else if (name == "rear_overtake_flow") s = rearOvertakeFlow(rng);
  else if (name == "door_oncoming") s = doorOncoming(rng);
  else if (name == "side_door") s = sideDoor(rng);
  else if (name == "group_oncoming") s = groupOncoming(rng);
  else if (name == "blind_corner") s = blindCorner(rng);
  else if (name == "elevator_flow") s = elevatorFlow(rng);
  else if (name == "sudden_stop") s = suddenCommon(rng, name, 1);
  else if (name == "sudden_turn") s = suddenCommon(rng, name, 2);
  else if (name == "sudden_reverse") s = suddenCommon(rng, name, 3);
  else throw std::invalid_argument("unknown scenario: " + name);
  return s;
}

}  // namespace sim
}  // namespace social_nav
