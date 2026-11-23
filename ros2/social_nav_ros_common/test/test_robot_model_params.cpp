#include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"
#include "social_nav_ros_common/robot_model_params.hpp"

using namespace social_nav_ros_common;

TEST(RobotModelParams, Defaults)
{
  auto node = std::make_shared<rclcpp::Node>("rmp_defaults");
  const auto spec = readFootprintSpec(node, "p");
  EXPECT_EQ(spec.type, "circle");
  EXPECT_DOUBLE_EQ(spec.radius, 0.3);
  EXPECT_TRUE(spec.points.empty());
  EXPECT_EQ(readMotionModel(node, "p"), "diff_drive");
  EXPECT_NE(social_nav::makeFootprint(spec), nullptr);
}

TEST(RobotModelParams, Overrides)
{
  rclcpp::NodeOptions opts;
  opts.parameter_overrides({
    {"q.footprint_type", "polygon"},
    {"q.robot_radius", 0.45},
    {"q.footprint", "[[0.3, 0.2], [0.3, -0.2], [-0.3, -0.2], [-0.3, 0.2]]"},
    {"q.motion_model", "omni"},
  });
  auto node = std::make_shared<rclcpp::Node>("rmp_over", opts);
  const auto spec = readFootprintSpec(node, "q");
  EXPECT_EQ(spec.type, "polygon");
  EXPECT_DOUBLE_EQ(spec.radius, 0.45);
  ASSERT_EQ(spec.points.size(), 4u);
  EXPECT_DOUBLE_EQ(spec.points[1].y, -0.2);
  EXPECT_DOUBLE_EQ(spec.points[2].x, -0.3);
  EXPECT_EQ(readMotionModel(node, "q"), "omni");
}

TEST(RobotModelParams, ParseErrors)
{
  EXPECT_THROW(parseFootprintString("[[1,2],[3]]"), std::invalid_argument);
  EXPECT_THROW(parseFootprintString("abc"), std::invalid_argument);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int r = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return r;
}
