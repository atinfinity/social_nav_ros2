#include <cmath>
#include <stdexcept>

#include "mini_test.hpp"
#include "social_nav_core/motion_model.hpp"

using namespace social_nav;

TEST(predict_constant_velocity) {
  Agent a{1, {1, 2}, {0.5, -1.0}, {0.01, 0, 0.01}, 0.3, 1.0};
  Agent b = predictConstantVelocity(a, 2.0);
  EXPECT_NEAR(b.pos.x, 2.0, 1e-12);
  EXPECT_NEAR(b.pos.y, 0.0, 1e-12);
  EXPECT_TRUE(b.pos_cov.xx > a.pos_cov.xx);
}

TEST(diff_drive_straight) {
  DiffDriveModel m;
  Pose2D p = m.integrate({1, 2, M_PI / 2}, {0.5, 0, 0}, 2.0);
  EXPECT_NEAR(p.x, 1.0, 1e-9);
  EXPECT_NEAR(p.y, 3.0, 1e-9);
  EXPECT_NEAR(p.theta, M_PI / 2, 1e-9);
}

TEST(diff_drive_rotate_in_place) {
  DiffDriveModel m;
  Pose2D p = m.integrate({1, 2, 0.1}, {0, 0, 1.0}, 0.5);
  EXPECT_NEAR(p.x, 1.0, 1e-12);
  EXPECT_NEAR(p.y, 2.0, 1e-12);
  EXPECT_NEAR(p.theta, 0.6, 1e-12);
  EXPECT_TRUE(m.canRotateInPlace());
}

TEST(diff_drive_arc) {
  DiffDriveModel m;
  // 半径 1 (v=1, w=1)、四分の一周(dt=pi/2)で (1,1), theta=pi/2
  Pose2D p = m.integrate({0, 0, 0}, {1.0, 0, 1.0}, M_PI / 2);
  EXPECT_NEAR(p.x, 1.0, 1e-9);
  EXPECT_NEAR(p.y, 1.0, 1e-9);
  EXPECT_NEAR(p.theta, M_PI / 2, 1e-9);
  // 刻みを細かくしても厳密解と一致(分割に依存しない)
  Pose2D q{0, 0, 0};
  for (int i = 0; i < 10; ++i) q = m.integrate(q, {1.0, 0, 1.0}, M_PI / 20);
  EXPECT_NEAR(q.x, 1.0, 1e-9);
  EXPECT_NEAR(q.y, 1.0, 1e-9);
}

TEST(diff_drive_sample_commands_within_limits) {
  DiffDriveModel m;
  VelocityLimits lim;
  SampleSpec spec;
  const double dt = 0.1;
  Twist2D cur{0.4, 0.0, 0.5};
  std::vector<Twist2D> out;
  m.sampleCommands(cur, lim, spec, dt, out);
  EXPECT_TRUE(out.size() == static_cast<std::size_t>(spec.v_samples * spec.w_samples));
  const double eps = 1e-9;
  for (const auto& c : out) {
    EXPECT_TRUE(c.vy == 0.0);
    EXPECT_TRUE(c.vx >= lim.v_min - eps && c.vx <= lim.v_max + eps);
    EXPECT_TRUE(std::fabs(c.wz) <= lim.w_max + eps);
    EXPECT_TRUE(std::fabs(c.vx - cur.vx) <= lim.acc_v * dt + eps);
    EXPECT_TRUE(std::fabs(c.wz - cur.wz) <= lim.acc_w * dt + eps);
  }
}

TEST(diff_drive_sample_commands_at_limit_edges) {
  DiffDriveModel m;
  VelocityLimits lim;
  SampleSpec spec;
  std::vector<Twist2D> out;
  m.sampleCommands({lim.v_max, 0, lim.w_max}, lim, spec, 0.1, out);
  for (const auto& c : out) {
    EXPECT_TRUE(c.vx <= lim.v_max + 1e-9);
    EXPECT_TRUE(c.wz <= lim.w_max + 1e-9);
    EXPECT_TRUE(c.vy == 0.0);
  }
  EXPECT_TRUE(!out.empty());
}

TEST(diff_drive_sample_commands_respects_acc_when_far_above_vmax) {
  DiffDriveModel m;
  VelocityLimits lim;
  SampleSpec spec;
  const double dt = 0.1;
  const Twist2D cur{lim.v_max + 10.0 * lim.acc_v * dt, 0.0, -(lim.w_max + 10.0 * lim.acc_w * dt)};
  std::vector<Twist2D> out;
  m.sampleCommands(cur, lim, spec, dt, out);
  EXPECT_TRUE(!out.empty());
  for (const auto& c : out) {
    EXPECT_TRUE(std::fabs(c.vx - cur.vx) <= lim.acc_v * dt + 1e-9);
    EXPECT_TRUE(std::fabs(c.wz - cur.wz) <= lim.acc_w * dt + 1e-9);
    EXPECT_TRUE(c.vx < cur.vx);   // 範囲へ向かって減速する側
  }
}

TEST(diff_drive_sample_commands_w_below_lower_limit_moves_up) {
  DiffDriveModel m;
  VelocityLimits lim;
  SampleSpec spec;
  const double dt = 0.1;
  const Twist2D cur{0.0, 0.0, -(lim.w_max + 10.0 * lim.acc_w * dt)};   // w が下限未満
  std::vector<Twist2D> out;
  m.sampleCommands(cur, lim, spec, dt, out);
  EXPECT_TRUE(!out.empty());
  for (const auto& c : out) {
    EXPECT_TRUE(c.wz > cur.wz);
    EXPECT_TRUE(std::fabs(c.wz - cur.wz) <= lim.acc_w * dt + 1e-9);
  }
}

TEST(make_motion_model) {
  auto m = makeMotionModel("diff_drive");
  EXPECT_TRUE(m != nullptr);
  EXPECT_TRUE(std::string(m->name()) == "diff_drive");
  EXPECT_THROW(makeMotionModel("omni"), std::invalid_argument);
  EXPECT_THROW(makeMotionModel("ackermann"), std::invalid_argument);
}

MINI_TEST_MAIN()
