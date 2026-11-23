#include <cstdint>
#include <vector>

#include "mini_test.hpp"
#include "social_nav_core/grid_rasterizer.hpp"

using namespace social_nav;

namespace {
const GridSpec kGrid{0.0, 0.0, 0.05, 200, 200};   // 10 m x 10 m
const CellWindow kAll{0, 0, 199, 199};

TrackedPeople one(double x, double y, double vx, double vy, double conf = 1.0) {
  TrackedPeople p;
  p.agents.push_back(Agent{1, {x, y}, {vx, vy}, {}, 0.3, conf});
  return p;
}
size_t idx(int i, int j) { return static_cast<size_t>(j) * kGrid.size_x + static_cast<size_t>(i); }
}  // namespace

TEST(raster_peak_position_and_value) {
  RasterParams prm;
  std::vector<uint8_t> cells(200 * 200, 0);
  // セル (100, 120) の中心に静止した人
  const TrackedPeople ppl = one(100.5 * 0.05, 120.5 * 0.05, 0, 0);
  rasterizeSocialCost(kGrid, kAll, ppl, prm, cells.data());
  size_t best = 0;
  for (size_t k = 0; k < cells.size(); ++k) if (cells[k] > cells[best]) best = k;
  EXPECT_TRUE(best == idx(100, 120));
  EXPECT_TRUE(cells[best] == prm.max_cost);
  EXPECT_TRUE(cells[idx(100, 121)] < cells[best]);
}

TEST(raster_keeps_lethal_and_unknown_cells) {
  RasterParams prm;
  std::vector<uint8_t> cells(200 * 200, 0);
  cells[idx(100, 100)] = 253;
  cells[idx(101, 100)] = 255;
  cells[idx(100, 101)] = 254;
  const TrackedPeople ppl = one(100.5 * 0.05, 100.5 * 0.05, 0, 0);
  rasterizeSocialCost(kGrid, kAll, ppl, prm, cells.data());
  EXPECT_TRUE(cells[idx(100, 100)] == 253);
  EXPECT_TRUE(cells[idx(101, 100)] == 255);
  EXPECT_TRUE(cells[idx(100, 101)] == 254);
  EXPECT_TRUE(cells[idx(99, 100)] > 0);
}

TEST(raster_max_merge_does_not_lower_cells) {
  RasterParams prm;
  std::vector<uint8_t> cells(200 * 200, 0);
  cells[idx(100, 100)] = 252;
  const TrackedPeople ppl = one(100.5 * 0.05, 100.5 * 0.05, 0, 0);
  rasterizeSocialCost(kGrid, kAll, ppl, prm, cells.data());
  EXPECT_TRUE(cells[idx(100, 100)] == 252);
}

TEST(raster_does_not_write_outside_window) {
  RasterParams prm;
  std::vector<uint8_t> cells(200 * 200, 0);
  const TrackedPeople ppl = one(100.5 * 0.05, 100.5 * 0.05, 0, 0);
  const CellWindow win{95, 95, 104, 104};
  rasterizeSocialCost(kGrid, win, ppl, prm, cells.data());
  size_t inside = 0, outside = 0;
  for (int j = 0; j < 200; ++j) {
    for (int i = 0; i < 200; ++i) {
      const bool in = i >= 95 && i <= 104 && j >= 95 && j <= 104;
      if (cells[idx(i, j)] != 0) (in ? inside : outside)++;
    }
  }
  EXPECT_TRUE(inside > 0);
  EXPECT_TRUE(outside == 0);
}

TEST(raster_skips_fast_people) {
  RasterParams prm;   // max_speed 0.3
  std::vector<uint8_t> cells(200 * 200, 0);
  const TrackedPeople ppl = one(5.0, 5.0, 1.2, 0.0);
  rasterizeSocialCost(kGrid, kAll, ppl, prm, cells.data());
  size_t nonzero = 0;
  for (uint8_t c : cells) nonzero += c != 0;
  EXPECT_TRUE(nonzero == 0);
  EXPECT_TRUE(!socialBounds(ppl, prm).valid);
}

TEST(raster_skips_low_confidence) {
  RasterParams prm;
  std::vector<uint8_t> cells(200 * 200, 0);
  const TrackedPeople ppl = one(5.0, 5.0, 0, 0, 0.1);
  rasterizeSocialCost(kGrid, kAll, ppl, prm, cells.data());
  size_t nonzero = 0;
  for (uint8_t c : cells) nonzero += c != 0;
  EXPECT_TRUE(nonzero == 0);
}

TEST(social_bounds_encloses_drawn_cells) {
  RasterParams prm;
  prm.prediction_time = 1.0;
  std::vector<uint8_t> cells(200 * 200, 0);
  const TrackedPeople ppl = one(4.0, 5.0, 0.25, 0.1);
  const Aabb box = socialBounds(ppl, prm);
  EXPECT_TRUE(box.valid);
  rasterizeSocialCost(kGrid, kAll, ppl, prm, cells.data());
  size_t drawn = 0, outside = 0;
  for (int j = 0; j < 200; ++j) {
    for (int i = 0; i < 200; ++i) {
      if (cells[idx(i, j)] == 0) continue;
      ++drawn;
      const double x = (i + 0.5) * 0.05, y = (j + 0.5) * 0.05;
      if (x < box.min_x || x > box.max_x || y < box.min_y || y > box.max_y) ++outside;
    }
  }
  EXPECT_TRUE(drawn > 0);
  EXPECT_TRUE(outside == 0);
  // 予測位置 (4.25, 5.1) も範囲に入る
  EXPECT_TRUE(box.max_x > 4.25);
}

TEST(social_bounds_empty_is_invalid) {
  RasterParams prm;
  EXPECT_TRUE(!socialBounds(TrackedPeople{}, prm).valid);
}

MINI_TEST_MAIN()
