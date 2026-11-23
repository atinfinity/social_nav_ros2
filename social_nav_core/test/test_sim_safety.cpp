#include "mini_test.hpp"
#include "sim/safety.hpp"

using namespace social_nav::sim;

TEST(cm_constants_match_yaml) {
  EXPECT_NEAR(kCmStopClearance, 0.15, 1e-12);
  EXPECT_NEAR(kCmSlowClearance, 0.5, 1e-12);
  EXPECT_NEAR(kCmSlowRatio, 0.4, 1e-12);
}

TEST(cm_stop_boundary) {
  for (SafetyMode m : {SafetyMode::CmStop, SafetyMode::Cm}) {
    EXPECT_TRUE(cmSafety(m, 0.149).stop);
    EXPECT_TRUE(cmSafety(m, 0.0).stop);
    EXPECT_TRUE(cmSafety(m, -0.1).stop);
    EXPECT_TRUE(!cmSafety(m, 0.15).stop);
    EXPECT_TRUE(!cmSafety(m, 3.0).stop);
  }
}

TEST(cm_slow_boundary) {
  EXPECT_NEAR(cmSafety(SafetyMode::Cm, 0.499).scale, 0.4, 1e-12);
  EXPECT_NEAR(cmSafety(SafetyMode::Cm, 0.15).scale, 0.4, 1e-12);
  EXPECT_NEAR(cmSafety(SafetyMode::Cm, 0.5).scale, 1.0, 1e-12);
  EXPECT_NEAR(cmSafety(SafetyMode::Cm, 2.0).scale, 1.0, 1e-12);
}

TEST(cm_stop_mode_has_no_slowdown) {
  EXPECT_NEAR(cmSafety(SafetyMode::CmStop, 0.3).scale, 1.0, 1e-12);
  EXPECT_TRUE(!cmSafety(SafetyMode::CmStop, 0.3).stop);
}

TEST(predictive_and_off_are_not_handled_here) {
  for (SafetyMode m : {SafetyMode::Predictive, SafetyMode::Off}) {
    const CmOutcome o = cmSafety(m, 0.0);
    EXPECT_TRUE(!o.stop);
    EXPECT_NEAR(o.scale, 1.0, 1e-12);
  }
}

TEST(parse_safety_mode) {
  SafetyMode m = SafetyMode::Off;
  EXPECT_TRUE(parseSafetyMode("predictive", m) && m == SafetyMode::Predictive);
  EXPECT_TRUE(parseSafetyMode("cm_stop", m) && m == SafetyMode::CmStop);
  EXPECT_TRUE(parseSafetyMode("cm", m) && m == SafetyMode::Cm);
  EXPECT_TRUE(parseSafetyMode("off", m) && m == SafetyMode::Off);
  m = SafetyMode::Cm;
  EXPECT_TRUE(!parseSafetyMode("CM", m) && m == SafetyMode::Cm);
  EXPECT_TRUE(!parseSafetyMode("", m));
}

MINI_TEST_MAIN()
