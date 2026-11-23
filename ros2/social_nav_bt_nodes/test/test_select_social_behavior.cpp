#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav2_msgs/msg/speed_limit.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "social_nav_bt_nodes/bt_compat.hpp"
#include "social_nav_msgs/msg/tracked_people.hpp"
#include "tf2_ros/buffer.h"

namespace
{
const char * kXml =
  "<root BTCPP_format=\"4\"><BehaviorTree ID=\"T\">"
  "<SelectSocialBehavior path=\"{path}\" behavior=\"{behavior}\"/>"
  "</BehaviorTree></root>";

nav_msgs::msg::Path straightPath()
{
  nav_msgs::msg::Path p;
  p.header.frame_id = "map";
  for (int i = 0; i <= 10; ++i) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = "map";
    ps.pose.position.x = 0.5 * i;
    ps.pose.orientation.w = 1.0;
    p.poses.push_back(ps);
  }
  return p;
}

struct Fixture
{
  explicit Fixture(const std::vector<rclcpp::Parameter> & overrides = {}, bool with_tf = true,
    const std::string & tf_parent = "map", const std::string & tf_child = "base_link")
  {
    rclcpp::NodeOptions opts;
    opts.parameter_overrides(overrides);
    node = std::make_shared<rclcpp::Node>("bt_host", opts);
    bb = BT::Blackboard::create();
    bb->set<rclcpp::Node::SharedPtr>("node", node);
    if (with_tf) {
      auto buf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
      geometry_msgs::msg::TransformStamped t;
      t.header.frame_id = tf_parent;
      t.child_frame_id = tf_child;
      t.transform.rotation.w = 1.0;
      buf->setTransform(t, "test", true);
      bb->set<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", buf);
    }
    bb->set<nav_msgs::msg::Path>("path", straightPath());
    factory.registerFromPlugin(SOCIAL_NAV_BT_PLUGIN_PATH);
    tree = factory.createTreeFromText(kXml, bb);
    sub = node->create_subscription<nav2_msgs::msg::SpeedLimit>(
      "/speed_limit", 10, [this](nav2_msgs::msg::SpeedLimit::ConstSharedPtr m) {limits.push_back(*m);});
    pub = node->create_publisher<social_nav_msgs::msg::TrackedPeople>(
      "/social_nav/tracked_people", 10);
  }

  // 被試験ノードの購読接続を待って publish する(配送待ちは tickUntil/tickFor が tick+spin で行う)。
  // node は外部 spin しない(people の反映は tick 内の spin_some だけで行われることを確かめる)。
  void publish(social_nav_msgs::msg::TrackedPeople msg)
  {
    const auto limit = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (pub->get_subscription_count() < 1 && std::chrono::steady_clock::now() < limit) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    msg.header.stamp = node->get_clock()->now();
    pub->publish(msg);
  }

  // behavior が expected になるまで 10 ms 刻みで tick する(上限 2 s)。
  bool tickUntil(const std::string & expected, int timeout_ms = 2000)
  {
    const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (true) {
      tree.tickOnce();
      if (behavior() == expected) {
        return true;
      }
      if (std::chrono::steady_clock::now() >= end) {
        return false;
      }
      rclcpp::spin_some(node);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  // 上限時間 tick を続け、全 tick が expected であったかを返す(変化しないことの確認用)。
  bool tickAllAre(const std::string & expected, int duration_ms = 500)
  {
    const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(duration_ms);
    bool all = true;
    while (std::chrono::steady_clock::now() < end) {
      tree.tickOnce();
      all = all && behavior() == expected;
      rclcpp::spin_some(node);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return all;
  }

  void spin(int ms = 200)
  {
    const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
    while (std::chrono::steady_clock::now() < end) {
      rclcpp::spin_some(node);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  std::string behavior() {return bb->get<std::string>("behavior");}

  rclcpp::Node::SharedPtr node;
  BT::BehaviorTreeFactory factory;
  BT::Blackboard::Ptr bb;
  BT::Tree tree;
  rclcpp::Subscription<nav2_msgs::msg::SpeedLimit>::SharedPtr sub;
  rclcpp::Publisher<social_nav_msgs::msg::TrackedPeople>::SharedPtr pub;
  std::vector<nav2_msgs::msg::SpeedLimit> limits;
};
}  // namespace

TEST(SelectSocialBehavior, NoPeopleProceedsAndSucceeds)
{
  Fixture f;
  EXPECT_EQ(f.tree.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(f.behavior(), "PROCEED");
  f.spin(50);
  EXPECT_TRUE(f.limits.empty());  // 100% のままなら送らない
}

TEST(SelectSocialBehavior, TfFailureProceedsAndSucceeds)
{
  Fixture f({}, false);
  social_nav_msgs::msg::TrackedPeople msg;
  msg.header.frame_id = "map";
  msg.header.stamp = f.node->get_clock()->now();
  social_nav_msgs::msg::TrackedPerson p;
  p.id = 1;
  p.position.x = 5.0;
  p.position.y = 5.0;
  p.radius = 0.3f;
  p.confidence = 1.0f;
  msg.people.push_back(p);
  f.tree.tickOnce();
  f.publish(msg);
  EXPECT_TRUE(f.tickAllAre("PROCEED"));  // TF が無いので受信後も PROCEED のまま
  EXPECT_EQ(f.tree.tickOnce(), BT::NodeStatus::SUCCESS);
}

TEST(SelectSocialBehavior, ThresholdsComeFromSocialBehaviorParams)
{
  // density_wait を極端に下げると、1 人いるだけで WAIT になる(閾値はパラメータだけで決まる)
  Fixture f({rclcpp::Parameter("social_behavior.density_wait", 0.01),
             rclcpp::Parameter("social_behavior.density_resume", 0.005)});
  social_nav_msgs::msg::TrackedPeople msg;
  msg.header.frame_id = "map";
  msg.header.stamp = f.node->get_clock()->now();
  social_nav_msgs::msg::TrackedPerson p;
  p.id = 1;
  p.position.x = 1.0;
  p.position.y = 2.5;  // 通路の外
  p.radius = 0.3f;
  p.confidence = 1.0f;
  msg.people.push_back(p);
  f.tree.tickOnce();  // 初回 tick で購読が作られる
  f.publish(msg);
  ASSERT_TRUE(f.tickUntil("WAIT"));
  EXPECT_EQ(f.tree.tickOnce(), BT::NodeStatus::SUCCESS);
  f.spin();
  ASSERT_FALSE(f.limits.empty());
  EXPECT_TRUE(f.limits.back().percentage);
  EXPECT_LT(f.limits.back().speed_limit, 100.0);
}

TEST(SelectSocialBehavior, StalePeopleProceed)
{
  Fixture f({rclcpp::Parameter("social_behavior.density_wait", 0.01),
             rclcpp::Parameter("social_behavior.density_resume", 0.005),
             rclcpp::Parameter("social_behavior.stale_timeout", 0.1)});
  social_nav_msgs::msg::TrackedPeople msg;
  msg.header.frame_id = "map";
  msg.header.stamp = f.node->get_clock()->now();
  social_nav_msgs::msg::TrackedPerson p;
  p.id = 1;
  p.position.x = 1.0;
  p.position.y = 2.5;
  p.radius = 0.3f;
  p.confidence = 1.0f;
  msg.people.push_back(p);
  f.tree.tickOnce();
  f.publish(msg);
  ASSERT_TRUE(f.tickUntil("WAIT"));  // まず受信が反映されることを確認
  std::this_thread::sleep_for(std::chrono::milliseconds(300));  // stale_timeout を超える
  EXPECT_EQ(f.tree.tickOnce(), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(f.behavior(), "PROCEED");
}

TEST(SelectSocialBehavior, GoalChangeResetsSelector)
{
  // min_dwell を長くして、reset が無ければ WAIT から出られない状況を作る
  Fixture f({rclcpp::Parameter("social_behavior.density_wait", 0.01),
             rclcpp::Parameter("social_behavior.density_resume", 0.005),
             rclcpp::Parameter("social_behavior.min_dwell", 30.0),
             // 既定 stale_timeout(0.5 s)だと tickAllAre の 500 ms 中に people が古くなり PROCEED になる
             rclcpp::Parameter("social_behavior.stale_timeout", 5.0)});
  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = "map";
  goal.pose.position.x = 5.0;
  goal.pose.orientation.w = 1.0;
  f.bb->set<geometry_msgs::msg::PoseStamped>("goal", goal);

  auto make = [&](double x, double y) {
      social_nav_msgs::msg::TrackedPeople msg;
      msg.header.frame_id = "map";
      social_nav_msgs::msg::TrackedPerson p;
      p.id = 1;
      p.position.x = x;
      p.position.y = y;
      p.radius = 0.3f;
      p.confidence = 1.0f;
      msg.people.push_back(p);
      return msg;
    };
  f.tree.tickOnce();
  f.publish(make(1.0, 2.5));
  ASSERT_TRUE(f.tickUntil("WAIT"));

  f.publish(make(9.0, 9.0));  // 密度 0 だが min_dwell のため WAIT のまま
  EXPECT_TRUE(f.tickAllAre("WAIT"));

  goal.pose.position.x = 6.0;  // ゴール変更 → reset
  f.bb->set<geometry_msgs::msg::PoseStamped>("goal", goal);
  f.publish(make(9.0, 9.0));
  EXPECT_TRUE(f.tickUntil("PROCEED"));
}

TEST(SelectSocialBehavior, FrameNamesComeFromNodeParams)
{
  // 既定と異なるフレーム名を node パラメータで与える。TF はその名前でのみ引ける。
  // 動く場合だけ WAIT になる(既定名 map/base_link を引くなら TF 失敗で PROCEED)。
  Fixture f({rclcpp::Parameter("global_frame", "odom"),
             rclcpp::Parameter("robot_base_frame", "robot_link"),
             rclcpp::Parameter("social_behavior.density_wait", 0.01),
             rclcpp::Parameter("social_behavior.density_resume", 0.005)}, true, "odom", "robot_link");
  auto path = straightPath();
  path.header.frame_id = "";  // global_frame(odom)で解決させる
  f.bb->set<nav_msgs::msg::Path>("path", path);

  social_nav_msgs::msg::TrackedPeople msg;
  msg.header.frame_id = "odom";
  social_nav_msgs::msg::TrackedPerson p;
  p.id = 1;
  p.position.x = 1.0;
  p.position.y = 2.5;
  p.radius = 0.3f;
  p.confidence = 1.0f;
  msg.people.push_back(p);
  f.tree.tickOnce();
  f.publish(msg);
  EXPECT_TRUE(f.tickUntil("WAIT"));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int r = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return r;
}
