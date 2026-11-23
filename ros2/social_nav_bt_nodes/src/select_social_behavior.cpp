#include "social_nav_bt_nodes/select_social_behavior.hpp"

#include <cmath>
#include <stdexcept>
#include <vector>

#include "tf2/exceptions.h"
#include "tf2/time.h"

namespace social_nav_bt
{
namespace
{

template<class T>
T param(const rclcpp::Node::SharedPtr & node, const std::string & name, const T & def)
{
  if (!node->has_parameter(name)) {
    node->declare_parameter(name, rclcpp::ParameterValue(def));
  }
  T v = def;
  node->get_parameter(name, v);
  return v;
}

double yawOf(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

social_nav::TacticalParams readTacticalParams(const rclcpp::Node::SharedPtr & node)
{
  using social_nav::TacticalParams;
  const TacticalParams d;
  const std::string p = "social_behavior.";
  TacticalParams t;
  t.corridor.lookahead = param(node, p + "corridor_lookahead", d.corridor.lookahead);
  t.corridor.half_width = param(node, p + "corridor_half_width", d.corridor.half_width);
  t.v_max = param(node, p + "v_max", d.v_max);
  t.density_radius = param(node, p + "density_radius", d.density_radius);
  t.density_low = param(node, p + "density_low", d.density_low);
  t.density_high = param(node, p + "density_high", d.density_high);
  t.min_ratio = param(node, p + "min_ratio", d.min_ratio);
  t.blocker_max_speed = param(node, p + "blocker_max_speed", d.blocker_max_speed);
  t.max_yield_time = param(node, p + "max_yield_time", d.max_yield_time);
  t.yield_cooldown = param(node, p + "yield_cooldown", d.yield_cooldown);
  t.follow_min_speed = param(node, p + "follow_min_speed", d.follow_min_speed);
  t.follow_heading_tol = param(node, p + "follow_heading_tol", d.follow_heading_tol);
  t.follow_gap = param(node, p + "follow_gap", d.follow_gap);
  t.density_wait = param(node, p + "density_wait", d.density_wait);
  t.density_resume = param(node, p + "density_resume", d.density_resume);
  t.max_wait_time = param(node, p + "max_wait_time", d.max_wait_time);
  t.min_dwell = param(node, p + "min_dwell", d.min_dwell);
  return t;
}

}  // namespace

SelectSocialBehavior::SelectSocialBehavior(const std::string & name, const NodeConfig & config)
: BT::SyncActionNode(name, config)
{
  if (!config.blackboard || !config.blackboard->get("node", node_) || !node_) {
    throw std::runtime_error("SelectSocialBehavior: blackboard has no 'node'");
  }
  static_cast<void>(config.blackboard->get("tf_buffer", tf_));  // 任意キー
  // Nav2 の BtActionServer は bt_navigator の global_frame/robot_base_frame を client node へ複製する
  global_frame_ = param<std::string>(node_, "global_frame", "map");
  robot_frame_ = param<std::string>(node_, "robot_base_frame", "base_link");
  bb_ = config.blackboard;
}

BT::PortsList SelectSocialBehavior::providedPorts()
{
  return {
    BT::InputPort<nav_msgs::msg::Path>("path", "Path being followed"),
    BT::InputPort<std::string>("people_topic", "/social_nav/tracked_people", "TrackedPeople topic"),
    BT::OutputPort<std::string>("behavior", "PROCEED/FOLLOW/YIELD/WAIT")};
}

void SelectSocialBehavior::lazyInit()
{
  if (initialized_) {
    return;
  }
  initialized_ = true;
  selector_ = std::make_unique<social_nav::TacticalSelector>(readTacticalParams(node_));
  const std::string p = "social_behavior.";
  const double stale = param(node_, p + "stale_timeout", 0.5);
  const double min_conf = param(node_, p + "min_confidence", 0.3);
  const auto speed_topic = param<std::string>(node_, p + "speed_limit_topic", "/speed_limit");
  std::string topic = "/social_nav/tracked_people";
  if (auto t = getInput<std::string>("people_topic")) {
    topic = t.value();
  }
  cb_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  cb_executor_.add_callback_group(cb_group_, node_->get_node_base_interface());
  people_ = std::make_unique<social_nav_ros_common::PeopleSubscriber>(
    node_, topic, stale, min_conf, cb_group_);
  speed_pub_ = node_->create_publisher<nav2_msgs::msg::SpeedLimit>(speed_topic, rclcpp::QoS(10));
}

void SelectSocialBehavior::publishSpeed(double ratio)
{
  const double pct = std::max(0.0, std::min(100.0, ratio * 100.0));
  // 変化 5% 未満は再送しない。ただし 100% への復帰は常に送る(減速のまま残さない)。
  const bool restore = pct >= 100.0 && last_percentage_ < 100.0;
  if (!restore && std::abs(pct - last_percentage_) < 5.0) {
    return;
  }
  nav2_msgs::msg::SpeedLimit msg;
  msg.header.stamp = node_->get_clock()->now();
  msg.percentage = true;
  msg.speed_limit = pct;
  speed_pub_->publish(msg);
  last_percentage_ = pct;
}

void SelectSocialBehavior::resetOnGoalChange()
{
  // Jazzy は同じ XML ならツリーを使い回すので、ゴールが変わっても状態が残る。
  // blackboard の goal(Nav2 が設定)を前回値と比べ、変わったら selector を初期化する。
  // goal が無い構成では何もしない(制限は docs/verification.md に記載)。
  geometry_msgs::msg::PoseStamped goal;
  if (!bb_ || !bb_->get("goal", goal)) {
    return;
  }
  const bool changed = has_goal_ && (goal.header.frame_id != last_goal_.header.frame_id ||
    goal.pose.position.x != last_goal_.pose.position.x ||
    goal.pose.position.y != last_goal_.pose.position.y ||
    goal.pose.orientation.z != last_goal_.pose.orientation.z ||
    goal.pose.orientation.w != last_goal_.pose.orientation.w);
  if (changed) {
    selector_->reset();
    // 送信済み速度の初期化。減速値が下流に残らないよう、100% 未満だったときは 100% を送る。
    publishSpeed(1.0);
    last_percentage_ = 100.0;
  }
  last_goal_ = goal;
  has_goal_ = true;
}

BT::NodeStatus SelectSocialBehavior::tick()
{
  lazyInit();
  cb_executor_.spin_some();  // people 購読コールバックを処理
  resetOnGoalChange();

  social_nav::TacticalDecision decision;  // 既定 = PROCEED・100%
  do {
    nav_msgs::msg::Path path_msg;
    if (!tf_ || !getInput("path", path_msg)) {
      break;
    }
    social_nav::TrackedPeople people = people_->get();
    const std::string frame = path_msg.header.frame_id.empty() ? global_frame_ :
      path_msg.header.frame_id;

    social_nav::Pose2D robot;
    try {
      const auto t = tf_->lookupTransform(frame, robot_frame_, tf2::TimePointZero);
      robot = {t.transform.translation.x, t.transform.translation.y, yawOf(t.transform.rotation)};
      if (!people.agents.empty() && !people.frame_id.empty() && people.frame_id != frame) {
        const auto pt = tf_->lookupTransform(frame, people.frame_id, tf2::TimePointZero);
        const double th = yawOf(pt.transform.rotation);
        const double c = std::cos(th), s = std::sin(th);
        const double tx = pt.transform.translation.x, ty = pt.transform.translation.y;
        for (auto & a : people.agents) {
          const auto pos = a.pos;
          const auto vel = a.vel;
          a.pos = {tx + c * pos.x - s * pos.y, ty + s * pos.x + c * pos.y};
          a.vel = {c * vel.x - s * vel.y, s * vel.x + c * vel.y};
        }
        people.frame_id = frame;
      }
    } catch (const tf2::TransformException &) {
      break;  // TF 失敗 → PROCEED
    }

    std::vector<social_nav::Pose2D> path;
    path.reserve(path_msg.poses.size());
    for (const auto & ps : path_msg.poses) {
      path.push_back({ps.pose.position.x, ps.pose.position.y, yawOf(ps.pose.orientation)});
    }
    social_nav::TacticalInput in{node_->get_clock()->now().seconds(), robot, &path, &people};
    decision = selector_->step(in);  // people が空でも状態機械は進める
    if (people.agents.empty()) {
      decision = social_nav::TacticalDecision{};  // 未受信・鮮度切れは PROCEED・100%
    }
  } while (false);

  setOutput("behavior", std::string(social_nav::toString(decision.behavior)));
  publishSpeed(decision.speed_ratio);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace social_nav_bt
