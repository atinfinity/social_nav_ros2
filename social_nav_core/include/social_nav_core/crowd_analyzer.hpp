#pragma once
#include <cstdint>
#include <vector>

#include "social_nav_core/types.hpp"

namespace social_nav {

// 戦術層の部品(状態を持たない幾何計算)
double crowdDensity(const TrackedPeople&, Vec2 center, double radius);             // 人/m²
double speedLimitRatio(double density, double low, double high, double min_ratio); // [min_ratio, 1]

struct CorridorParams { double lookahead{2.5}; double half_width{0.5}; };          // half_width の目安 = circumscribedRadius + 0.2
// speed_along: 経路接線方向成分(同方向が正)
struct PersonOnPath { uint32_t id; double dist_along; double speed_along; double speed; };

// ロボットを経路へ射影した点から lookahead までの通路(半幅 half_width)内の人を、近い順に返す
std::vector<PersonOnPath> peopleOnPath(const std::vector<Pose2D>& path, const Pose2D& robot,
                                       const TrackedPeople&, const CorridorParams&);

}  // namespace social_nav
