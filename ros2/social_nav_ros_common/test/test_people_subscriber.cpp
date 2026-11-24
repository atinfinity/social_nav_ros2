#include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"
#include "social_nav_ros_common/people_subscriber.hpp"

using namespace social_nav_ros_common;

namespace
{
social_nav::TrackedPeople make(double stamp, double conf)
{
  social_nav::TrackedPeople p;
  p.stamp = stamp;
  social_nav::Agent a;
  a.id = 1;
  a.confidence = conf;
  p.agents.push_back(a);
  return p;
}
}  // namespace

TEST(FreshPeople, FreshKept)
{
  EXPECT_EQ(freshPeople(make(10.0, 0.9), 10.3, 0.5, 0.3).agents.size(), 1u);
}

TEST(FreshPeople, StaleEmpty)
{
  EXPECT_TRUE(freshPeople(make(10.0, 0.9), 10.6, 0.5, 0.3).agents.empty());
}

TEST(FreshPeople, LowConfidenceDropped)
{
  EXPECT_TRUE(freshPeople(make(10.0, 0.1), 10.1, 0.5, 0.3).agents.empty());
}

TEST(PeopleSubscriber, EmptyBeforeReceive)
{
  auto node = std::make_shared<rclcpp::Node>("people_sub_test");
  PeopleSubscriber sub(node, "people", 0.5, 0.3);
  EXPECT_TRUE(sub.get().agents.empty());
  EXPECT_FALSE(sub.valid(10.0));
}

TEST(PeopleSubscriber, ReturnsThenStale)
{
  auto node = std::make_shared<rclcpp::Node>("people_sub_test2");
  PeopleSubscriber sub(node, "people", 0.5, 0.3);
  sub.update(toMsg(make(10.0, 0.9)));
  EXPECT_EQ(sub.get(10.2).agents.size(), 1u);
  EXPECT_TRUE(sub.get(11.0).agents.empty());
}

TEST(PeopleSubscriber, ValidOnlyWhenReceivedAndFresh)
{
  auto node = std::make_shared<rclcpp::Node>("people_sub_test3");
  PeopleSubscriber sub(node, "people", 0.5, 0.3);
  EXPECT_FALSE(sub.valid(10.2));
  social_nav::TrackedPeople empty;
  empty.stamp = 10.0;
  sub.update(toMsg(empty));  // 人 0 人でも受信済み・鮮度内なら真
  EXPECT_TRUE(sub.valid(10.2));
  EXPECT_FALSE(sub.valid(10.6));
  EXPECT_TRUE(sub.get(10.6).agents.empty());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int r = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return r;
}
