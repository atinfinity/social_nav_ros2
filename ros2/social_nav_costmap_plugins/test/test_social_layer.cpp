#include <gtest/gtest.h>

#include <atomic>
#include <memory>

#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "social_nav_msgs/msg/tracked_people.hpp"
#include "tf2_ros/buffer.h"

namespace {

class SocialLayerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override
  {
    node_ = std::make_shared<nav2_util::LifecycleNode>("social_layer_test");
    clock_ = node_->get_clock();
    tf_ = std::make_shared<tf2_ros::Buffer>(clock_);
    costmap_ = std::make_unique<nav2_costmap_2d::LayeredCostmap>("map", false, false);
    costmap_->resizeMap(100, 100, 0.1, 0.0, 0.0);
    loader_ = std::make_unique<pluginlib::ClassLoader<nav2_costmap_2d::Layer>>(
      "nav2_costmap_2d", "nav2_costmap_2d::Layer");
    pub_ = node_->create_publisher<social_nav_msgs::msg::TrackedPeople>(
      "/social_nav/tracked_people", 10);
    probe_ = node_->create_subscription<social_nav_msgs::msg::TrackedPeople>(
      "/social_nav/tracked_people", 10,
      [this](social_nav_msgs::msg::TrackedPeople::ConstSharedPtr) {++received_;});
  }

  void TearDown() override
  {
    probe_.reset();
    layer_.reset();
    loader_.reset();
  }

  void makeLayer()
  {
    layer_ = loader_->createSharedInstance("social_nav_costmap_plugins::SocialLayer");
    layer_->initialize(costmap_.get(), "social", tf_.get(), node_, nullptr);
  }

  void publish(double x, double y, double vx = 0.0, double vy = 0.0, double conf = 1.0)
  {
    social_nav_msgs::msg::TrackedPeople msg;
    msg.header.frame_id = "map";
    msg.header.stamp = clock_->now();
    social_nav_msgs::msg::TrackedPerson p;
    p.id = 1;
    p.position.x = x;
    p.position.y = y;
    p.velocity.x = vx;
    p.velocity.y = vy;
    p.position_covariance = {0.0, 0.0, 0.0, 0.0};
    p.radius = 0.3F;
    p.confidence = static_cast<float>(conf);
    msg.people.push_back(p);
    // 購読の接続(probe + layer)を待つ。layer が購読しないテストもあるので上限付き。
    for (int i = 0; i < 200 && pub_->get_subscription_count() < 2; ++i) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_GT(pub_->get_subscription_count(), 0U);
    msg.header.stamp = clock_->now();
    const int before = received_;
    pub_->publish(msg);
    // 配送されるまで(上限付き)spin。固定時間待ちは並列実行の負荷で不足する
    for (int i = 0; i < 400 && received_ == before; ++i) {
      rclcpp::spin_some(node_->get_node_base_interface());
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    for (int i = 0; i < 5; ++i) {  // layer 側の購読コールバックも処理させる
      rclcpp::spin_some(node_->get_node_base_interface());
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }

  // 1 サイクル分(updateBounds → updateCosts を全域)
  void cycle()
  {
    double x0 = 1e30, y0 = 1e30, x1 = -1e30, y1 = -1e30;
    layer_->updateBounds(0, 0, 0, &x0, &y0, &x1, &y1);
    last_ = {x0, y0, x1, y1};
    layer_->updateCosts(*costmap_->getCostmap(), 0, 0, 100, 100);
  }

  unsigned char cost(double x, double y)
  {
    unsigned i, j;
    costmap_->getCostmap()->worldToMap(x, y, i, j);
    return costmap_->getCostmap()->getCost(i, j);
  }

  std::shared_ptr<nav2_util::LifecycleNode> node_;
  rclcpp::Clock::SharedPtr clock_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::unique_ptr<nav2_costmap_2d::LayeredCostmap> costmap_;
  std::unique_ptr<pluginlib::ClassLoader<nav2_costmap_2d::Layer>> loader_;
  std::shared_ptr<nav2_costmap_2d::Layer> layer_;
  rclcpp::Publisher<social_nav_msgs::msg::TrackedPeople>::SharedPtr pub_;
  rclcpp::Subscription<social_nav_msgs::msg::TrackedPeople>::SharedPtr probe_;
  std::atomic<int> received_{0};
  struct Box { double x0, y0, x1, y1; } last_{};
};

TEST_F(SocialLayerTest, LoadsViaPluginlib)
{
  makeLayer();
  ASSERT_NE(layer_, nullptr);
  EXPECT_FALSE(layer_->isClearable());
}

TEST_F(SocialLayerTest, NoPeopleDrawsNothing)
{
  makeLayer();
  cycle();
  EXPECT_EQ(cost(5.0, 5.0), nav2_costmap_2d::FREE_SPACE);
}

TEST_F(SocialLayerTest, DrawsCostAroundPerson)
{
  makeLayer();
  publish(5.0, 5.0);
  cycle();
  const auto c = cost(5.0, 5.0);
  EXPECT_GT(c, 100);
  EXPECT_LE(c, 200);
  EXPECT_EQ(cost(8.0, 8.0), nav2_costmap_2d::FREE_SPACE);
}

TEST_F(SocialLayerTest, MaxMergeKeepsLethalAndUnknown)
{
  makeLayer();
  auto * m = costmap_->getCostmap();
  unsigned i, j;
  m->worldToMap(5.0, 5.0, i, j);
  m->setCost(i, j, nav2_costmap_2d::LETHAL_OBSTACLE);
  m->setCost(i + 1, j, nav2_costmap_2d::NO_INFORMATION);
  m->setCost(i, j + 1, 250);
  publish(5.0, 5.0);
  cycle();
  EXPECT_EQ(m->getCost(i, j), nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_EQ(m->getCost(i + 1, j), nav2_costmap_2d::NO_INFORMATION);
  EXPECT_EQ(m->getCost(i, j + 1), 250);
}

TEST_F(SocialLayerTest, FastPersonAndLowConfidenceNotDrawn)
{
  makeLayer();
  publish(5.0, 5.0, 1.0, 0.0);
  cycle();
  EXPECT_EQ(cost(5.0, 5.0), nav2_costmap_2d::FREE_SPACE);
  publish(5.0, 5.0, 0.0, 0.0, 0.1);
  cycle();
  EXPECT_EQ(cost(5.0, 5.0), nav2_costmap_2d::FREE_SPACE);
}

TEST_F(SocialLayerTest, BoundsIncludePreviousAabb)
{
  makeLayer();
  publish(3.0, 3.0);
  cycle();
  publish(7.0, 7.0);
  cycle();
  // 今回の窓は前回(3,3 付近)と今回(7,7 付近)の両方を含む
  EXPECT_LT(last_.x0, 3.0);
  EXPECT_GT(last_.x1, 7.0);
  EXPECT_LT(last_.y0, 3.0);
  EXPECT_GT(last_.y1, 7.0);
}

TEST_F(SocialLayerTest, DisabledDrawsNothing)
{
  node_->declare_parameter("social.enabled", false);
  makeLayer();
  publish(5.0, 5.0);
  cycle();
  EXPECT_EQ(cost(5.0, 5.0), nav2_costmap_2d::FREE_SPACE);
}

}  // namespace
