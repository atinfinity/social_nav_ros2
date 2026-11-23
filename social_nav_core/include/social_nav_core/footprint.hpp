#pragma once
#include <memory>
#include <string>
#include <vector>

#include "social_nav_core/types.hpp"

namespace social_nav {

class Footprint {
 public:
  virtual ~Footprint() = default;
  virtual double inscribedRadius() const = 0;
  virtual double circumscribedRadius() const = 0;
  // ロボット外形から点 p までの距離。内部は負
  virtual double clearance(const Pose2D& robot, Vec2 p) const = 0;
  // 障害物コストを照会する点(ワールド座標)を out に追加する。円は中心 1 点
  virtual void costQueryPoints(const Pose2D& robot, std::vector<Vec2>& out) const = 0;
};

class CircleFootprint final : public Footprint {
 public:
  explicit CircleFootprint(double radius);
  double inscribedRadius() const override;
  double circumscribedRadius() const override;
  double clearance(const Pose2D& robot, Vec2 p) const override;
  void costQueryPoints(const Pose2D& robot, std::vector<Vec2>& out) const override;

 private:
  double radius_;
};

struct FootprintSpec { std::string type{"circle"}; double radius{0.3}; std::vector<Vec2> points; };
std::unique_ptr<Footprint> makeFootprint(const FootprintSpec& spec);     // "circle"。"polygon" は未実装で std::invalid_argument

}  // namespace social_nav
