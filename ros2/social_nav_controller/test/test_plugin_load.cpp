#include <gtest/gtest.h>

#include <memory>

#include "nav2_core/controller.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

class RosEnv : public ::testing::Environment
{
public:
  void SetUp() override {rclcpp::init(0, nullptr);}
  void TearDown() override {rclcpp::shutdown();}
};
const auto * const env = ::testing::AddGlobalTestEnvironment(new RosEnv);

TEST(SocialControllerPlugin, LoadsViaPluginlib)
{
  pluginlib::ClassLoader<nav2_core::Controller> loader("nav2_core", "nav2_core::Controller");
  nav2_core::Controller::Ptr c;
  ASSERT_NO_THROW(c = loader.createSharedInstance("social_nav_controller::SocialController"));
  EXPECT_NE(c, nullptr);
}

TEST(SocialControllerPlugin, UnsupportedMotionModelFailsConfigure)
{
  pluginlib::ClassLoader<nav2_core::Controller> loader("nav2_core", "nav2_core::Controller");
  auto c = loader.createSharedInstance("social_nav_controller::SocialController");
  rclcpp::NodeOptions opts;
  opts.parameter_overrides({{"ctl.motion_model", "omni"}});
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("ctl_node", opts);
  EXPECT_THROW(c->configure(node, "ctl", nullptr, nullptr), std::invalid_argument);
}

TEST(SocialControllerPlugin, SetSpeedLimitBeforeConfigureDoesNotThrow)
{
  pluginlib::ClassLoader<nav2_core::Controller> loader("nav2_core", "nav2_core::Controller");
  auto c = loader.createSharedInstance("social_nav_controller::SocialController");
  EXPECT_NO_THROW(c->setSpeedLimit(50.0, true));
  EXPECT_NO_THROW(c->setSpeedLimit(0.3, false));
}
