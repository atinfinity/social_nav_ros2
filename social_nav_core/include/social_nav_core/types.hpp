#pragma once
#include <cstdint>
#include <string>
#include <vector>

namespace social_nav {

struct Vec2 { double x{0}, y{0}; };
struct Point3 { float x, y, z; };
struct Pose2D { double x{0}, y{0}, theta{0}; };
struct Twist2D { double vx{0}, vy{0}, wz{0}; };       // vy は差動二輪では常に 0。Omni 用に最初から持つ
struct Cov2 { double xx{0}, xy{0}, yy{0}; };
struct Agent { uint32_t id; Vec2 pos; Vec2 vel; Cov2 pos_cov; double radius{0.3}; double confidence{1.0}; };
struct TrackedPeople { double stamp{0}; std::string frame_id; std::vector<Agent> agents; };

// 等速予測。共分散は dt に応じて膨らませる(速度の不確かさ 0.3 m/s を仮定: xx,yy += (0.3 dt)^2)
Agent predictConstantVelocity(const Agent& a, double dt);

}  // namespace social_nav
