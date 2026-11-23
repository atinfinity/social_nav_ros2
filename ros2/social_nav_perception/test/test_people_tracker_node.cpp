#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <memory>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "social_nav_msgs/msg/tracked_people.hpp"
#include "social_nav_perception/people_tracker_node.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

using namespace std::chrono_literals;

namespace
{
// base_link 座標の円柱(半径 0.15 m、z 0.4-1.2 m)を 1 本含む点群
sensor_msgs::msg::PointCloud2 makeCloud(const rclcpp::Time & stamp)
{
  sensor_msgs::msg::PointCloud2 c;
  c.header.frame_id = "lidar";
  c.header.stamp = stamp;
  std::vector<std::array<float, 3>> pts;
  for (int k = 0; k < 24; ++k) {
    const double a = 2.0 * M_PI * k / 24.0;
    for (int h = 0; h < 5; ++h) {
      pts.push_back({static_cast<float>(3.0 + 0.15 * std::cos(a)), static_cast<float>(1.0 + 0.15 * std::sin(a)),
                     static_cast<float>(0.4 + 0.2 * h)});
    }
  }
  sensor_msgs::PointCloud2Modifier mod(c);
  mod.setPointCloud2FieldsByString(1, "xyz");
  mod.resize(pts.size());
  sensor_msgs::PointCloud2Iterator<float> x(c, "x"), y(c, "y"), z(c, "z");
  for (const auto & p : pts) {
    *x = p[0]; *y = p[1]; *z = p[2];
    ++x; ++y; ++z;
  }
  return c;
}
}  // namespace

// ノードを 3 秒回し、静止円柱が確定して配信されること(異常終了しないこと)を確認する
TEST(PeopleTrackerNode, RunsThreeSecondsAndPublishesConfirmedPerson)
{
  auto node = std::make_shared<social_nav_perception::PeopleTrackerNode>();
  auto helper = std::make_shared<rclcpp::Node>("people_tracker_test_helper");

  tf2_ros::StaticTransformBroadcaster br(helper);
  std::vector<geometry_msgs::msg::TransformStamped> tfs(2);
  tfs[0].header.frame_id = "odom";
  tfs[0].child_frame_id = "base_link";
  tfs[0].transform.rotation.w = 1.0;
  tfs[1].header.frame_id = "base_link";
  tfs[1].child_frame_id = "lidar";
  tfs[1].transform.rotation.w = 1.0;
  br.sendTransform(tfs);

  auto cloud_pub = helper->create_publisher<sensor_msgs::msg::PointCloud2>("points", rclcpp::SensorDataQoS());
  social_nav_msgs::msg::TrackedPeople last;
  int received = 0;
  auto sub = helper->create_subscription<social_nav_msgs::msg::TrackedPeople>(
    "/social_nav/tracked_people", 10, [&](social_nav_msgs::msg::TrackedPeople::SharedPtr m) {
      last = *m;
      ++received;
    });
  auto timer = helper->create_wall_timer(100ms, [&]() {cloud_pub->publish(makeCloud(helper->now()));});

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(helper);
  const auto end = std::chrono::steady_clock::now() + 3s;
  while (std::chrono::steady_clock::now() < end) {
    exec.spin_some(50ms);
  }

  EXPECT_GE(received, 10);
  EXPECT_EQ(last.header.frame_id, "odom");
  ASSERT_EQ(last.people.size(), 1u);
  EXPECT_NEAR(last.people[0].position.x, 3.0, 0.2);
  EXPECT_NEAR(last.people[0].position.y, 1.0, 0.2);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int r = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return r;
}
