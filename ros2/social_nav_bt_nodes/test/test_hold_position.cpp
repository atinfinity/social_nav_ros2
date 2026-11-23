#include <gtest/gtest.h>

#include <string>

#include "social_nav_bt_nodes/bt_compat.hpp"

namespace
{
BT::BehaviorTreeFactory makeFactory()
{
  BT::BehaviorTreeFactory factory;
  factory.registerFromPlugin(SOCIAL_NAV_BT_PLUGIN_PATH);
  return factory;
}

BT::NodeStatus tickWith(const std::string & behavior)
{
  auto factory = makeFactory();
  const std::string xml =
    "<root BTCPP_format=\"4\"><BehaviorTree ID=\"T\"><HoldPosition behavior=\"" + behavior +
    "\"/></BehaviorTree></root>";
  auto tree = factory.createTreeFromText(xml);
  return tree.tickOnce();
}
}  // namespace

TEST(HoldPosition, RegisteredInFactory)
{
  auto factory = makeFactory();
  const auto & m = factory.manifests();
  EXPECT_TRUE(m.count("HoldPosition"));
  EXPECT_TRUE(m.count("SelectSocialBehavior"));
}

TEST(HoldPosition, YieldAndWaitAreRunning)
{
  EXPECT_EQ(tickWith("YIELD"), BT::NodeStatus::RUNNING);
  EXPECT_EQ(tickWith("WAIT"), BT::NodeStatus::RUNNING);
}

TEST(HoldPosition, OthersAreFailure)
{
  EXPECT_EQ(tickWith("PROCEED"), BT::NodeStatus::FAILURE);
  EXPECT_EQ(tickWith("FOLLOW"), BT::NodeStatus::FAILURE);
  EXPECT_EQ(tickWith("garbage"), BT::NodeStatus::FAILURE);
}

TEST(HoldPosition, ChangesFromRunningToFailureAndNeverSucceeds)
{
  auto factory = makeFactory();
  auto bb = BT::Blackboard::create();
  bb->set<std::string>("behavior", "WAIT");
  auto tree = factory.createTreeFromText(
    "<root BTCPP_format=\"4\"><BehaviorTree ID=\"T\"><HoldPosition behavior=\"{behavior}\"/>"
    "</BehaviorTree></root>", bb);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::RUNNING);
  bb->set<std::string>("behavior", "PROCEED");
  EXPECT_EQ(tree.tickOnce(), BT::NodeStatus::FAILURE);
}
