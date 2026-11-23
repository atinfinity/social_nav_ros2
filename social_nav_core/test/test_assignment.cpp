#include <random>
#include <vector>

#include "mini_test.hpp"
#include "social_nav_core/assignment.hpp"

using namespace social_nav;
using Matrix = std::vector<std::vector<double>>;
using Mask = std::vector<std::vector<bool>>;

namespace {
// 全探索: 割り当て数最大 -> 総コスト最小
void brute(const Matrix& c, const Mask& f, std::size_t i, std::vector<bool>& used, int cnt, double sum,
           int& best_cnt, double& best_sum) {
  if (i == c.size()) {
    if (cnt > best_cnt || (cnt == best_cnt && sum < best_sum)) { best_cnt = cnt; best_sum = sum; }
    return;
  }
  brute(c, f, i + 1, used, cnt, sum, best_cnt, best_sum);
  for (std::size_t j = 0; j < c[i].size(); ++j) {
    if (used[j] || (!f.empty() && f[i][j])) continue;
    used[j] = true;
    brute(c, f, i + 1, used, cnt + 1, sum + c[i][j], best_cnt, best_sum);
    used[j] = false;
  }
}

void check(const Matrix& c, const Mask& f) {
  const std::vector<int> a = solveAssignment(c, f);
  EXPECT_TRUE(a.size() == c.size());
  std::vector<bool> used(c.empty() ? 0 : c[0].size(), false);
  int cnt = 0;
  double sum = 0;
  for (std::size_t i = 0; i < a.size(); ++i) {
    if (a[i] < 0) continue;
    EXPECT_TRUE(!used[static_cast<std::size_t>(a[i])]);
    EXPECT_TRUE(f.empty() || !f[i][static_cast<std::size_t>(a[i])]);
    used[static_cast<std::size_t>(a[i])] = true;
    ++cnt;
    sum += c[i][static_cast<std::size_t>(a[i])];
  }
  int best_cnt = -1;
  double best_sum = 0;
  std::vector<bool> u2(used.size(), false);
  brute(c, f, 0, u2, 0, 0.0, best_cnt, best_sum);
  EXPECT_TRUE(cnt == best_cnt);
  EXPECT_NEAR(sum, best_sum, 1e-9);
}
}  // namespace

TEST(matches_brute_force_on_random_matrices) {
  std::mt19937 rng(12345);
  std::uniform_real_distribution<double> d(-5.0, 50.0);
  std::uniform_int_distribution<int> dim(1, 6);
  for (int t = 0; t < 300; ++t) {
    const int r = dim(rng), k = dim(rng);
    Matrix c(r, std::vector<double>(k));
    Mask f(r, std::vector<bool>(k, false));
    const bool with_mask = t % 2 == 1;
    for (int i = 0; i < r; ++i)
      for (int j = 0; j < k; ++j) {
        c[i][j] = d(rng);
        if (with_mask) f[i][j] = rng() % 3 == 0;
      }
    check(c, with_mask ? f : Mask{});
  }
}

TEST(rectangular_rows_more_than_cols) {
  const Matrix c = {{1, 9}, {9, 1}, {5, 5}};
  const std::vector<int> a = solveAssignment(c);
  EXPECT_TRUE(a[0] == 0);
  EXPECT_TRUE(a[1] == 1);
  EXPECT_TRUE(a[2] == -1);
}

TEST(rectangular_rows_fewer_than_cols) {
  const Matrix c = {{9, 1, 5}, {9, 2, 1}};
  const std::vector<int> a = solveAssignment(c);
  EXPECT_TRUE(a[0] == 1);
  EXPECT_TRUE(a[1] == 2);
}

TEST(all_forbidden_gives_no_assignment) {
  const Matrix c = {{1, 2}, {3, 4}};
  const Mask f = {{true, true}, {true, true}};
  for (int x : solveAssignment(c, f)) EXPECT_TRUE(x == -1);
}

TEST(forbidden_pair_is_skipped_even_if_cheapest) {
  const Matrix c = {{0, 10}, {1, 10}};
  const Mask f = {{true, false}, {false, true}};
  const std::vector<int> a = solveAssignment(c, f);
  EXPECT_TRUE(a[0] == 1 || a[0] == -1);
  EXPECT_TRUE(a[1] == 0 || a[1] == -1);
  EXPECT_TRUE(a[0] == 1 && a[1] == 0);
}

TEST(empty_inputs) {
  EXPECT_TRUE(solveAssignment({}).empty());
  const std::vector<int> a = solveAssignment(Matrix(3), Mask{});
  EXPECT_TRUE(a.size() == 3 && a[0] == -1 && a[1] == -1 && a[2] == -1);
}

TEST(ties_are_deterministic) {
  const Matrix c(4, std::vector<double>(4, 1.0));
  const std::vector<int> first = solveAssignment(c);
  for (int i = 0; i < 10; ++i) EXPECT_TRUE(solveAssignment(c) == first);
  int cnt = 0;
  for (int x : first) cnt += x >= 0;
  EXPECT_TRUE(cnt == 4);
}

namespace {
struct Stat { int cnt = 0; double sum = 0; bool valid = true; };
Stat evaluate(const Matrix& c, const Mask& f, const std::vector<int>& a) {
  Stat s;
  const std::size_t nc = c.empty() ? 0 : c[0].size();
  std::vector<bool> used(nc, false);
  if (a.size() != c.size()) s.valid = false;
  for (std::size_t i = 0; i < a.size() && i < c.size(); ++i) {
    if (a[i] < 0) continue;
    const std::size_t j = static_cast<std::size_t>(a[i]);
    if (j >= nc || used[j] || (!f.empty() && f[i][j])) { s.valid = false; continue; }
    used[j] = true;
    ++s.cnt;
    s.sum += c[i][j];
  }
  return s;
}
Matrix transpose(const Matrix& c) {
  Matrix t(c[0].size(), std::vector<double>(c.size()));
  for (std::size_t i = 0; i < c.size(); ++i)
    for (std::size_t j = 0; j < c[i].size(); ++j) t[j][i] = c[i][j];
  return t;
}
Mask transposeMask(const Mask& f) {
  if (f.empty()) return f;
  Mask t(f[0].size(), std::vector<bool>(f.size()));
  for (std::size_t i = 0; i < f.size(); ++i)
    for (std::size_t j = 0; j < f[i].size(); ++j) t[j][i] = f[i][j];
  return t;
}
// 妥当性と、転置入力との割り当て数・総コスト一致を確認する。
void checkTranspose(const Matrix& c, const Mask& f) {
  const Stat a = evaluate(c, f, solveAssignment(c, f));
  const Matrix ct = transpose(c);
  const Mask ft = transposeMask(f);
  const Stat b = evaluate(ct, ft, solveAssignment(ct, ft));
  EXPECT_TRUE(a.valid);
  EXPECT_TRUE(b.valid);
  EXPECT_TRUE(a.cnt == b.cnt);
  EXPECT_NEAR(a.sum, b.sum, 1e-9);
}
}  // namespace

TEST(matches_brute_force_with_many_ties) {
  std::mt19937 rng(777);
  std::uniform_int_distribution<int> dim(1, 6), val(0, 3);
  for (int t = 0; t < 300; ++t) {
    const int r = dim(rng), k = dim(rng);
    Matrix c(r, std::vector<double>(k));
    Mask f(r, std::vector<bool>(k, false));
    const bool with_mask = t % 3 != 0;
    const bool sparse = t % 3 == 2;  // 禁止率 0.7
    for (int i = 0; i < r; ++i)
      for (int j = 0; j < k; ++j) {
        c[i][j] = val(rng);
        if (with_mask) f[i][j] = sparse ? (rng() % 10 < 7) : (rng() % 3 == 0);
      }
    check(c, with_mask ? f : Mask{});
  }
}

TEST(all_equal_square_gives_identity) {
  for (int n = 1; n <= 8; ++n) {
    const Matrix c(n, std::vector<double>(n, 2.5));
    const std::vector<int> first = solveAssignment(c);
    for (int i = 0; i < n; ++i) EXPECT_TRUE(first[i] == i);
    for (int r = 0; r < 10; ++r) EXPECT_TRUE(solveAssignment(c) == first);
  }
}

TEST(large_128_valid_and_transpose_consistent) {
  std::mt19937 rng(2024);
  std::uniform_real_distribution<double> d(0.0, 100.0), dn(-20.0, 100.0);
  const int n = 128;
  Matrix c(n, std::vector<double>(n)), cn(n, std::vector<double>(n));
  Mask f90(n, std::vector<bool>(n));
  for (int i = 0; i < n; ++i)
    for (int j = 0; j < n; ++j) {
      c[i][j] = d(rng);
      cn[i][j] = dn(rng);
      f90[i][j] = (rng() % 10) < 9;
    }
  checkTranspose(c, {});
  checkTranspose(cn, {});
  checkTranspose(cn, f90);
  checkTranspose(c, f90);
}

TEST(rectangular_large_and_degenerate_shapes) {
  std::mt19937 rng(99);
  std::uniform_real_distribution<double> d(-5.0, 50.0);
  auto make = [&](int r, int k) {
    Matrix c(r, std::vector<double>(k));
    for (auto& row : c) for (auto& x : row) x = d(rng);
    return c;
  };
  checkTranspose(make(64, 128), {});
  checkTranspose(make(128, 64), {});
  for (int n : {1, 2, 7, 30}) {
    checkTranspose(make(1, n), {});
    checkTranspose(make(n, 1), {});
  }
  const Matrix r1 = {{5, 3, 4}};
  EXPECT_TRUE(solveAssignment(r1)[0] == 1);
  const Matrix c1 = {{5}, {3}, {4}};
  const std::vector<int> a = solveAssignment(c1);
  EXPECT_TRUE(a[0] == -1 && a[1] == 0 && a[2] == -1);
}

TEST(all_columns_allowed_for_one_row_only) {
  // 行 2 だけが全列に許可され、他の行は列 0 のみ(行 0, 1)。最大数は 2。
  const Matrix c = {{1, 1, 1}, {2, 2, 2}, {9, 1, 5}};
  const Mask f = {{false, true, true}, {false, true, true}, {false, false, false}};
  check(c, f);
  // 全列が行 1 だけに許可される。
  const Mask g = {{true, true, true}, {false, false, false}, {true, true, true}};
  const std::vector<int> a = solveAssignment(c, g);
  EXPECT_TRUE(a[0] == -1 && a[1] == 0 && a[2] == -1);
  check(c, g);
}

TEST(both_sides_of_heap_linear_density_boundary) {
  // 辺数 x 8 <= 行数 x 列数 で最小頂点の取り出しが切り替わる。同じ行列で禁止率を両側に振る。
  std::mt19937 rng(31337);
  std::uniform_real_distribution<double> d(-5.0, 50.0), u(0.0, 1.0);
  const int n = 40;
  Matrix c(n, std::vector<double>(n));
  std::vector<std::vector<double>> r(n, std::vector<double>(n));
  for (int i = 0; i < n; ++i)
    for (int j = 0; j < n; ++j) { c[i][j] = d(rng); r[i][j] = u(rng); }
  for (double rate : {0.0, 0.5, 0.8, 0.86, 0.88, 0.9, 0.95}) {
    Mask f(n, std::vector<bool>(n));
    for (int i = 0; i < n; ++i)
      for (int j = 0; j < n; ++j) f[i][j] = r[i][j] < rate;
    checkTranspose(c, f);
  }
}

MINI_TEST_MAIN()
