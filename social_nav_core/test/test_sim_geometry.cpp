// 視線の遮蔽の幾何。線分交差(端点接触 = 交差)、lineOfSight、visibleAny
#include "mini_test.hpp"
#include "sim/social_force.hpp"

using namespace social_nav;
using namespace social_nav::sim;

TEST(segments_cross) {
  EXPECT_TRUE(segmentsIntersect({{0, 0}, {2, 2}}, {{0, 2}, {2, 0}}));
  EXPECT_TRUE(!segmentsIntersect({{0, 0}, {1, 1}}, {{0, 2}, {2, 0.5}}));
}

TEST(segments_parallel_and_collinear) {
  EXPECT_TRUE(!segmentsIntersect({{0, 0}, {2, 0}}, {{0, 1}, {2, 1}}));      // 平行
  EXPECT_TRUE(segmentsIntersect({{0, 0}, {2, 0}}, {{1, 0}, {3, 0}}));       // 同一直線で重なる
  EXPECT_TRUE(!segmentsIntersect({{0, 0}, {1, 0}}, {{2, 0}, {3, 0}}));      // 同一直線で離れる
  EXPECT_TRUE(segmentsIntersect({{0, 0}, {1, 0}}, {{1, 0}, {3, 0}}));       // 同一直線で端点だけ接する
}

TEST(segments_endpoint_touch) {
  EXPECT_TRUE(segmentsIntersect({{0, 0}, {2, 0}}, {{1, 0}, {1, 2}}));       // T 字接触(片方の端点が他方の途中)
  EXPECT_TRUE(segmentsIntersect({{0, 0}, {1, 1}}, {{1, 1}, {2, 0}}));       // 端点どうし
  EXPECT_TRUE(!segmentsIntersect({{0, 0}, {2, 0}}, {{1, 0.001}, {1, 2}}));  // わずかに離れる
}

TEST(line_of_sight_behind_wall) {
  const std::vector<Segment> walls{{{2, -1}, {2, 1}}};
  EXPECT_TRUE(!lineOfSight({0, 0}, {4, 0}, walls));
  EXPECT_TRUE(lineOfSight({0, 0}, {4, 3}, walls));        // 壁の上を通る
  EXPECT_TRUE(lineOfSight({0, 0}, {1.5, 0}, walls));      // 壁の手前
  EXPECT_TRUE(!lineOfSight({0, 0}, {4, 0.5}, {{{2, 1}, {2, 0.25}}}));   // 壁の端点側を通る
  EXPECT_TRUE(!lineOfSight({0, 0}, {2, 1}, walls));       // 壁の端点ちょうどは遮蔽
  EXPECT_TRUE(lineOfSight({0, 0}, {4, 0}, {}));           // 壁なし
}

TEST(visible_any_through_opening) {
  const std::vector<Segment> walls{{{2, -1}, {2, 1}}};
  // 中心の視線は壁に当たるが、半径の片側(+法線側)は壁の上端を越えて通る
  EXPECT_TRUE(!lineOfSight({0, 0}, {4, 1.8}, walls));
  EXPECT_TRUE(visibleAny({0, 0}, {4, 1.8}, 0.3, walls));
  EXPECT_TRUE(!visibleAny({0, 0}, {4, 0}, 0.3, walls));   // 3 本とも壁に当たる
  EXPECT_TRUE(visibleAny({0, 0}, {4, 3}, 0.3, walls));    // 中心が通る
}

MINI_TEST_MAIN()
