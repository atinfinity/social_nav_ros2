#include <cmath>
#include <stdexcept>

#include "mini_test.hpp"
#include "social_nav_core/footprint.hpp"

using namespace social_nav;

TEST(circle_clearance_is_distance_minus_radius) {
  CircleFootprint f(0.3);
  EXPECT_NEAR(f.clearance({0, 0, 0}, {1.0, 0.0}), 0.7, 1e-12);
  EXPECT_NEAR(f.clearance({1, 1, 0}, {4.0, 5.0}), 4.7, 1e-12);
  EXPECT_NEAR(f.clearance({0, 0, 0}, {0.1, 0.0}), -0.2, 1e-12);  // 内部は負
  EXPECT_NEAR(f.inscribedRadius(), 0.3, 1e-12);
  EXPECT_NEAR(f.circumscribedRadius(), 0.3, 1e-12);
}

TEST(circle_clearance_independent_of_heading) {
  CircleFootprint f(0.3);
  const double c0 = f.clearance({0, 0, 0.0}, {1.0, 2.0});
  for (double th : {0.5, 1.7, -2.0, 3.0}) EXPECT_NEAR(f.clearance({0, 0, th}, {1.0, 2.0}), c0, 1e-12);
}

TEST(circle_cost_query_points_single_center) {
  CircleFootprint f(0.3);
  std::vector<Vec2> pts;
  f.costQueryPoints({2.0, 3.0, 1.0}, pts);
  EXPECT_TRUE(pts.size() == 1);
  EXPECT_NEAR(pts[0].x, 2.0, 1e-12);
  EXPECT_NEAR(pts[0].y, 3.0, 1e-12);
  f.costQueryPoints({0, 0, 0}, pts);  // 追加であること
  EXPECT_TRUE(pts.size() == 2);
}

TEST(make_footprint) {
  FootprintSpec spec;
  auto f = makeFootprint(spec);
  EXPECT_TRUE(f != nullptr);
  EXPECT_NEAR(f->circumscribedRadius(), 0.3, 1e-12);
  FootprintSpec poly;
  poly.type = "polygon";
  poly.points = {{0.3, 0.2}, {-0.3, 0.2}, {-0.3, -0.2}, {0.3, -0.2}};
  EXPECT_THROW(makeFootprint(poly), std::invalid_argument);
}

MINI_TEST_MAIN()
