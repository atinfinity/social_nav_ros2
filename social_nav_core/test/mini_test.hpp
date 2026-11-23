#pragma once
// 最小のテストフレームワーク。失敗数を終了コードにする。
#include <cmath>
#include <cstdio>
#include <functional>
#include <string>
#include <vector>

namespace mini_test {
struct Registry {
  std::vector<std::pair<std::string, std::function<void()>>> tests;
  int failures{0};
  static Registry& get() { static Registry r; return r; }
};
struct Registrar {
  Registrar(const char* name, std::function<void()> fn) { Registry::get().tests.emplace_back(name, std::move(fn)); }
};
inline int runAll() {
  auto& r = Registry::get();
  for (auto& t : r.tests) {
    const int before = r.failures;
    t.second();
    std::printf("[%s] %s\n", r.failures == before ? " OK " : "FAIL", t.first.c_str());
  }
  std::printf("%zu tests, %d failures\n", r.tests.size(), r.failures);
  return r.failures == 0 ? 0 : 1;
}
}  // namespace mini_test

#define TEST(name)                                                                  \
  static void test_##name();                                                        \
  static mini_test::Registrar registrar_##name(#name, test_##name);                 \
  static void test_##name()

#define EXPECT_TRUE(cond)                                                           \
  do {                                                                              \
    if (!(cond)) {                                                                  \
      ++mini_test::Registry::get().failures;                                        \
      std::printf("  %s:%d: EXPECT_TRUE(%s) failed\n", __FILE__, __LINE__, #cond);  \
    }                                                                               \
  } while (0)

#define EXPECT_NEAR(a, b, tol)                                                      \
  do {                                                                              \
    const double a_ = (a), b_ = (b), t_ = (tol);                                    \
    if (!(std::fabs(a_ - b_) <= t_)) {                                              \
      ++mini_test::Registry::get().failures;                                        \
      std::printf("  %s:%d: EXPECT_NEAR(%s, %s, %s) failed: %g vs %g\n", __FILE__,  \
                  __LINE__, #a, #b, #tol, a_, b_);                                  \
    }                                                                               \
  } while (0)

#define EXPECT_THROW(stmt, ExType)                                                  \
  do {                                                                              \
    bool thrown_ = false;                                                           \
    try { stmt; } catch (const ExType&) { thrown_ = true; } catch (...) {}          \
    if (!thrown_) {                                                                 \
      ++mini_test::Registry::get().failures;                                        \
      std::printf("  %s:%d: EXPECT_THROW(%s, %s) failed\n", __FILE__, __LINE__,     \
                  #stmt, #ExType);                                                  \
    }                                                                               \
  } while (0)

#define MINI_TEST_MAIN() int main() { return mini_test::runAll(); }
