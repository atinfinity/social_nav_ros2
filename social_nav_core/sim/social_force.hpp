#pragma once
// 歩行者モデル: Social Force(design 6.2)。ROS 非依存。
#include <cstdint>
#include <vector>

#include "social_nav_core/types.hpp"

namespace social_nav {
namespace sim {

struct Segment { Vec2 a, b; };

struct SfmParams {
  double A{2.1};        // m/s^2
  double B{0.3};        // m
  double tau{0.5};      // s
  double lambda{0.35};  // 異方性 w(phi) = lambda + (1-lambda)(1+cos phi)/2
  double k_contact{500.0};   // 体の接触(重なり)に対するばね項 [1/s^2]。設計式にない補助項(重なりを 1 cm 台に抑える)
  double max_speed{2.0};
};

struct Pedestrian {
  uint32_t id{0};
  Vec2 pos, vel, goal, home;
  double v0{1.3};
  double radius{0.3};
  bool frozen{false};   // 動かない人(力は及ぼす)
  bool loop{false};     // 目標到達で home と goal を入れ替えて往復
  // 以下は PedModel(sfm2)だけが読む。legacy の SocialForceWorld は読まない。
  int kind{0};          // 0 通常 / 1 家族 / 2 歩きスマホ(意味づけは PedModel 側)
  int group_id{-1};     // 群れ番号。-1 は群れなし
  double attention{1.0};  // 他者・ロボットへの注意係数(相互作用項に掛かる)。1 が通常
};

struct RobotBody { Vec2 pos, vel; double radius{0.3}; };

Vec2 nearestPointOnSegment(Vec2 p, const Segment& s);
double distanceToWalls(Vec2 p, const std::vector<Segment>& walls);   // 壁なしなら大きな値
// 壁の反発。q = walls[wi] 上の p の最近点。q が線分の端点で、それより前の線分(walls[0..wi-1])の最近点と
// 一致するとき true(= 複数の線分が共有する角を壁の力で 2 回以上数えないために、2 本目以降は飛ばす)。端点でなければ常に false
bool isRepeatedWallCorner(Vec2 p, const std::vector<Segment>& walls, std::size_t wi, Vec2 q);

// 視線の遮蔽。壁は厚さ 0 の線分。端点に触れたら交差とみなす(閉区間。遮蔽側 = 見えない側に倒す)
bool segmentsIntersect(const Segment& s, const Segment& t);
bool lineOfSight(Vec2 from, Vec2 to, const std::vector<Segment>& walls);   // どの壁とも交差しなければ true
// from から、中心 center・半径 radius の人が見えるか。中心と、視線に垂直な ±radius の 2 点のどれかが通れば true
bool visibleAny(Vec2 from, Vec2 center, double radius, const std::vector<Segment>& walls);

class SocialForceWorld {
 public:
  SocialForceWorld(SfmParams prm, std::vector<Segment> walls);
  // robot が nullptr なら歩行者はロボットを無視する(--peds-ignore-robot)
  void step(std::vector<Pedestrian>& peds, const RobotBody* robot, double dt) const;
  const std::vector<Segment>& walls() const { return walls_; }

 private:
  SfmParams p_;
  std::vector<Segment> walls_;
};

}  // namespace sim
}  // namespace social_nav
