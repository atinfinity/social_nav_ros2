#include <gtest/gtest.h>

#include <memory>

#include "critic_data_adapter.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_mppi_controller/critic_function.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "social_nav_msgs/msg/tracked_people.hpp"

namespace
{
class RclcppEnv : public ::testing::Environment
{
public:
  void SetUp() override {rclcpp::init(0, nullptr);}
  void TearDown() override {rclcpp::shutdown();}
};
::testing::Environment * const env = ::testing::AddGlobalTestEnvironment(new RclcppEnv);
}  // namespace

TEST(Adapter, ViewAndAddCost)
{
  mppi::models::State state;
  mppi::models::Trajectories traj;
  traj.reset(3, 4);
  traj.x(1, 2) = 1.5f;
  traj.y(1, 2) = -2.0f;
  traj.yaws(1, 3) = 0.5f;
  mppi::models::Path path;
  geometry_msgs::msg::Pose goal;
  xt::xtensor<float, 1> costs = xt::zeros<float>({3});
  float dt = 0.05f;
  mppi::CriticData data{state, traj, path, goal, costs, dt, false, nullptr, nullptr, {}, {}};

  const auto v = social_nav_mppi_critics::makeView(data, 1);
  EXPECT_EQ(v.n, 4u);
  EXPECT_EQ(v.stride, 1);
  EXPECT_FLOAT_EQ(v.x[2 * v.stride], 1.5f);
  EXPECT_FLOAT_EQ(v.y[2 * v.stride], -2.0f);
  EXPECT_FLOAT_EQ(v.yaw[3 * v.stride], 0.5f);

  social_nav_mppi_critics::addCost(data, 2, 1.25);
  social_nav_mppi_critics::addCost(data, 2, 0.75);
  EXPECT_FLOAT_EQ(costs(2), 2.0f);
  EXPECT_FLOAT_EQ(costs(0), 0.0f);
}

TEST(SocialCritic, PluginLoadConfigureAndScore)
{
  pluginlib::ClassLoader<mppi::critics::CriticFunction> loader(
    "nav2_mppi_controller", "mppi::critics::CriticFunction");
  auto critic = loader.createUniqueInstance("mppi::critics::SocialCritic");
  ASSERT_NE(critic, nullptr);

  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("test_costmap");
  costmap->configure();
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("ctrl");
  mppi::ParametersHandler handler(node);
  critic->on_configure(node, "ctrl", "SocialCritic", costmap, &handler);
  EXPECT_EQ(critic->getName(), "SocialCritic");

  // 人(1.0, 0.0, 静止)に向かう軌道(batch 0)と、離れる軌道(batch 1)
  auto pub_node = std::make_shared<rclcpp::Node>("people_pub");
  auto pub = pub_node->create_publisher<social_nav_msgs::msg::TrackedPeople>("/social_nav/tracked_people", 10);
  social_nav_msgs::msg::TrackedPeople msg;
  msg.header.frame_id = costmap->getGlobalFrameID();
  msg.header.stamp = pub_node->now();
  social_nav_msgs::msg::TrackedPerson person;
  person.id = 1;
  person.position.x = 1.0;
  person.radius = 0.3;
  person.confidence = 1.0;
  person.position_covariance = {0.01, 0.0, 0.0, 0.01};
  msg.people.push_back(person);

  mppi::models::State state;
  mppi::models::Trajectories traj;
  traj.reset(2, 6);
  for (int k = 0; k < 6; ++k) {
    traj.x(0, k) = 0.15f * k;
    traj.x(1, k) = -0.15f * k;
  }
  mppi::models::Path path;
  geometry_msgs::msg::Pose goal;
  xt::xtensor<float, 1> costs = xt::zeros<float>({2});
  float dt = 0.1f;
  mppi::CriticData data{state, traj, path, goal, costs, dt, false, nullptr, nullptr, {}, {}};

  // 未受信なら何も足さない
  critic->score(data);
  EXPECT_FLOAT_EQ(costs(0), 0.0f);
  EXPECT_FLOAT_EQ(costs(1), 0.0f);

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node->get_node_base_interface());
  exec.add_node(pub_node);
  for (int i = 0; i < 20 && costs(0) == 0.0f; ++i) {
    msg.header.stamp = pub_node->now();
    pub->publish(msg);
    exec.spin_some(std::chrono::milliseconds(50));
    critic->score(data);
  }
  EXPECT_GT(costs(0), 0.0f);
  EXPECT_GT(costs(0), costs(1));
  exec.remove_node(pub_node);
  critic.reset();
}
