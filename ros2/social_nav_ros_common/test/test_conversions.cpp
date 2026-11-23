#include <gtest/gtest.h>

#include "social_nav_ros_common/conversions.hpp"

using namespace social_nav_ros_common;

TEST(Conversions, AgentRoundTrip)
{
  social_nav::Agent a;
  a.id = 7;
  a.pos = {1.5, -2.0};
  a.vel = {0.25, 0.5};
  a.pos_cov = {0.04, 0.01, 0.09};
  a.radius = 0.25;
  a.confidence = 0.75;
  const auto b = toCore(toMsg(a));
  EXPECT_EQ(b.id, 7u);
  EXPECT_DOUBLE_EQ(b.pos.x, 1.5);
  EXPECT_DOUBLE_EQ(b.pos.y, -2.0);
  EXPECT_DOUBLE_EQ(b.vel.x, 0.25);
  EXPECT_DOUBLE_EQ(b.vel.y, 0.5);
  EXPECT_DOUBLE_EQ(b.pos_cov.xx, 0.04);
  EXPECT_DOUBLE_EQ(b.pos_cov.xy, 0.01);
  EXPECT_DOUBLE_EQ(b.pos_cov.yy, 0.09);
  EXPECT_FLOAT_EQ(static_cast<float>(b.radius), 0.25f);
  EXPECT_FLOAT_EQ(static_cast<float>(b.confidence), 0.75f);
}

TEST(Conversions, PeopleRoundTrip)
{
  social_nav::TrackedPeople p;
  p.stamp = 12.5;
  p.frame_id = "odom";
  social_nav::Agent a;
  a.id = 1;
  p.agents.push_back(a);
  a.id = 2;
  p.agents.push_back(a);
  const auto m = toMsg(p);
  EXPECT_EQ(m.header.frame_id, "odom");
  EXPECT_EQ(m.header.stamp.sec, 12);
  EXPECT_EQ(m.header.stamp.nanosec, 500000000u);
  const auto q = toCore(m);
  EXPECT_NEAR(q.stamp, 12.5, 1e-9);
  EXPECT_EQ(q.frame_id, "odom");
  ASSERT_EQ(q.agents.size(), 2u);
  EXPECT_EQ(q.agents[1].id, 2u);
}

TEST(Conversions, EmptyPeople)
{
  EXPECT_TRUE(toCore(social_nav_msgs::msg::TrackedPeople()).agents.empty());
}
