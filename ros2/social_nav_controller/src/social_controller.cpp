#include "social_nav_controller/social_controller.hpp"

#include <cmath>
#include <stdexcept>
#include <vector>

#include "nav2_core/controller_exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "social_nav_controller/controller_utils.hpp"
#include "social_nav_core/footprint.hpp"
#include "social_nav_core/motion_model.hpp"
#include "social_nav_ros_common/robot_model_params.hpp"

namespace social_nav_controller {

namespace {
double yawOf(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}
}  // namespace

void SocialController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent;
  auto node = parent.lock();
  if (!node) {
    throw std::runtime_error("SocialController: parent node expired");
  }
  using social_nav_ros_common::detail::readParam;
  using Ptr = std::shared_ptr<rclcpp_lifecycle::LifecycleNode>;
  name_ = name;
  logger_ = node->get_logger();
  clock_ = node->get_clock();
  tf_ = tf;
  costmap_ros_ = costmap_ros;

  // MVP: diff_drive 以外は configure で失敗させる(他のパラメータ・購読を作る前に検査)
  const std::string model_name = social_nav_ros_common::readMotionModel(node, name);
  if (model_name != "diff_drive") {
    throw std::invalid_argument(
            "SocialController: motion_model '" + model_name + "' is not supported (diff_drive only)");
  }
  auto model = std::shared_ptr<const social_nav::MotionModel>(social_nav::makeMotionModel(model_name));
  auto footprint = std::shared_ptr<const social_nav::Footprint>(
    social_nav::makeFootprint(social_nav_ros_common::readFootprintSpec(node, name)));

  auto d = [&](const std::string & key, double def) {return readParam<Ptr, double>(node, name + "." + key, def);};
  auto i = [&](const std::string & key, int def) {return static_cast<int>(readParam<Ptr, int64_t>(node, name + "." + key, def));};

  const std::string people_topic =
    readParam<Ptr, std::string>(node, name + ".people_topic", "/social_nav/tracked_people");
  const double stale_timeout = d("stale_timeout", 0.5);
  path_lookahead_ = d("path_lookahead", 3.0);
  stuck_timeout_ = d("stuck_timeout", 2.0);

  social_nav::ControllerParams p;
  p.limits.v_min = d("v_min", p.limits.v_min);
  p.limits.v_max = d("v_max", p.limits.v_max);
  p.limits.vy_max = d("vy_max", p.limits.vy_max);
  p.limits.w_max = d("w_max", p.limits.w_max);
  p.limits.acc_v = d("acc_v", p.limits.acc_v);
  p.limits.acc_vy = d("acc_vy", p.limits.acc_vy);
  p.limits.acc_w = d("acc_w", p.limits.acc_w);
  p.limits.min_turning_radius = d("min_turning_radius", p.limits.min_turning_radius);
  p.samples.v_samples = i("v_samples", p.samples.v_samples);
  p.samples.vy_samples = i("vy_samples", p.samples.vy_samples);
  p.samples.w_samples = i("w_samples", p.samples.w_samples);
  p.horizon = d("horizon", p.horizon);
  p.dt = d("dt", p.dt);
  p.w_path = d("w_path", p.w_path);
  p.w_goal = d("w_goal", p.w_goal);
  p.w_obstacle = d("w_obstacle", p.w_obstacle);
  p.w_social = d("w_social", p.w_social);
  p.w_speed = d("w_speed", p.w_speed);
  p.social.prox.sigma = d("sigma", p.social.prox.sigma);
  p.social.prox.velocity_gain = d("velocity_gain", p.social.prox.velocity_gain);
  p.social.prox.rear_ratio = d("rear_ratio", p.social.prox.rear_ratio);
  p.social.prox.static_speed = d("static_speed", p.social.prox.static_speed);
  p.social.prox.cutoff = d("cutoff", p.social.prox.cutoff);
  p.social.collision_margin = d("collision_margin", p.social.collision_margin);
  p.social.collision_cost = d("collision_cost", p.social.collision_cost);
  p.social.time_decay_tau = d("time_decay_tau", p.social.time_decay_tau);
  p.social.step_subsample = i("step_subsample", p.social.step_subsample);
  p.social.max_range = d("max_range", p.social.max_range);
  p.social.min_confidence = d("min_confidence", p.social.min_confidence);
  // accel_rollout(既定 false): people_valid かつ人なしが accel_clear_time 秒続いた周期だけ、加速継続の軌道も評価する。
  // 入力の欠落(未受信、鮮度切れ、TF 失敗)は people_valid 偽として渡す。実機で有効にする前提の 1 つを満たしただけで、
  // 一度も見えていない人(壁の裏、視野外)や accel_clear_time を超える無言の欠落は区別できず、開口部で人と出会う課題が残る
  // ので、実機では有効にしない
  p.accel_rollout = readParam<Ptr, bool>(node, name + ".accel_rollout", p.accel_rollout);
  p.accel_clear_time = d("accel_clear_time", p.accel_clear_time);
  {
    bool disabled = false;
    p.accel_rollout = social_nav_controller::resolveAccelRollout(p.accel_rollout, p.accel_clear_time, disabled);
    if (disabled) {
      RCLCPP_WARN(logger_, "accel_rollout requires accel_clear_time > 0 (got %.3f); accel_rollout disabled",
                  p.accel_clear_time);
    }
  }
  if (p.accel_rollout) {
    const double freq = readParam<Ptr, double>(node, "controller_frequency", 20.0);
    if (social_nav_controller::accelPeriodMismatch(freq, p.dt)) {
      RCLCPP_WARN(logger_, "controller period (1/%.3f Hz) differs from dt=%.3f: accel_clear_time counts dt per call",
                  freq, p.dt);
    }
  }
  // door_yield(既定 false): 狭所点の判定に使う静的障害物コストを、地図(door_map_topic)の距離場から sim と同じ式で作る。
  // sim の証拠は accel_rollout と door_yield の両方 on の組だけ。地図に載らない開口部は検出できない等の前提が残るので、
  // 実機では有効にしない(docs/sim_results.md の door_yield の節)。door_* の閾値は core の既定のまま
  p.door_yield = readParam<Ptr, bool>(node, name + ".door_yield", p.door_yield);
  door_yield_ = p.door_yield;
  if (door_yield_) {
    const std::string map_topic = readParam<Ptr, std::string>(node, name + ".door_map_topic", "map");
    door_occupied_thresh_ = i("door_occupied_thresh", door_occupied_thresh_);
    const double freq = readParam<Ptr, double>(node, "controller_frequency", 20.0);
    if (social_nav_controller::accelPeriodMismatch(freq, p.dt)) {
      RCLCPP_WARN(logger_, "controller period (1/%.3f Hz) differs from dt=%.3f: door_hold_max and door_cooldown count dt per call",
                  freq, p.dt);
    }
    if (!p.accel_rollout) {
      RCLCPP_WARN(logger_, "door_yield is on but accel_rollout is off: the sim evidence is only for both on");
    }
    door_map_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic, rclcpp::QoS(1).transient_local().reliable(),
      [this](const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
        auto field = buildDistanceField(*msg, door_occupied_thresh_);
        {
          std::lock_guard<std::mutex> lk(door_map_mutex_);
          door_field_ = field;
          door_map_frame_ = msg->header.frame_id;
        }
        RCLCPP_INFO(logger_, "door_yield: map received (%ux%u, %.3f m/cell, frame %s)", msg->info.width,
                    msg->info.height, msg->info.resolution, msg->header.frame_id.c_str());
      });
  }
  v_max_ = p.limits.v_max;

  controller_ = std::make_unique<social_nav::SamplingController>(p, model, footprint);
  people_ = std::make_unique<social_nav_ros_common::PeopleSubscriber>(
    node, people_topic, stale_timeout, p.social.min_confidence);
  reset();
}

void SocialController::cleanup()
{
  door_map_sub_.reset();
  {
    std::lock_guard<std::mutex> lk(door_map_mutex_);
    door_field_.reset();
  }
  people_.reset();
  controller_.reset();
  plan_ = nav_msgs::msg::Path();
}

void SocialController::activate() {}

void SocialController::deactivate() {}

void SocialController::reset()
{
  colliding_ = false;
  speed_limit_ratio_ = 1.0;
}

void SocialController::setPlan(const nav_msgs::msg::Path & path)
{
  plan_ = path;
  colliding_ = false;
}

void SocialController::setSpeedLimit(compat::SpeedLimitArg speed_limit, compat::PercentageArg percentage)
{
  speed_limit_ratio_ = compat::speedLimitRatio(speed_limit, percentage, v_max_);
}

geometry_msgs::msg::TwistStamped SocialController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose, const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker *)
{
  if (!controller_) {
    throw nav2_core::ControllerException("SocialController is not configured");
  }
  if (plan_.poses.empty()) {
    throw nav2_core::InvalidPath("SocialController: empty plan");
  }
  const double speed_limit_ratio = speed_limit_ratio_.load();  // 1 回だけ読む
  const std::string global_frame = costmap_ros_->getGlobalFrameID();

  // 1. 経路を global_frame へ(同じフレームなら変換なし)
  double tx = 0.0, ty = 0.0, tyaw = 0.0;
  if (!plan_.header.frame_id.empty() && plan_.header.frame_id != global_frame) {
    try {
      const auto t = tf_->lookupTransform(global_frame, plan_.header.frame_id, tf2::TimePointZero);
      tx = t.transform.translation.x;
      ty = t.transform.translation.y;
      tyaw = yawOf(t.transform.rotation);
    } catch (const std::exception & e) {
      throw nav2_core::ControllerTFError(std::string("SocialController: path transform failed: ") + e.what());
    }
  }
  const double c = std::cos(tyaw), s = std::sin(tyaw);
  std::vector<social_nav::Pose2D> path;
  path.reserve(plan_.poses.size());
  for (const auto & ps : plan_.poses) {
    const auto & p = ps.pose.position;
    path.push_back({c * p.x - s * p.y + tx, s * p.x + c * p.y + ty, yawOf(ps.pose.orientation) + tyaw});
  }
  const social_nav::Pose2D robot{pose.pose.position.x, pose.pose.position.y, yawOf(pose.pose.orientation)};
  path = prunePath(path, robot, path_lookahead_);

  // 3. people(鮮度切れは空)。フレームが違えば global_frame へ
  auto people = people_->get();
  bool people_valid = people_->valid();  // 未受信・鮮度切れで偽
  if (!people.agents.empty() && !people.frame_id.empty() && people.frame_id != global_frame) {
    try {
      const auto t = tf_->lookupTransform(global_frame, people.frame_id, tf2::TimePointZero);
      people = transformPeople(people, t.transform.translation.x, t.transform.translation.y,
                               yawOf(t.transform.rotation), global_frame);
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "people transform failed, ignoring people: %s", e.what());
      people.agents.clear();
      people_valid = false;
    }
  }

  // 2, 3. 障害物コスト(local costmap)とコア呼び出し
  const auto obstacles = makeObstacleCostFn(*costmap_ros_->getCostmap());
  const social_nav::Twist2D vel{velocity.linear.x, velocity.linear.y, velocity.angular.z};
  // door_yield off: 空のまま(従来と同じ引数)。on: 地図未受信・TF 失敗でも空にせず常に 0 を返す関数を渡す
  // (空だと core が人を含む obstacles に戻る)
  social_nav::ObstacleCostFn static_obstacles;
  if (door_yield_) {
    static_obstacles = makeZeroCostFn();
    std::shared_ptr<const DistanceField> field;
    std::string map_frame;
    {
      std::lock_guard<std::mutex> lk(door_map_mutex_);
      field = door_field_;
      map_frame = door_map_frame_;
    }
    if (!field) {
      RCLCPP_WARN_THROTTLE(logger_, *clock_, 5000, "door_yield: map not received yet, no narrow point");
    } else {
      try {
        // 経路・姿勢は global_frame(odom)、距離場は map。p_map = R p_odom + t
        const auto t = tf_->lookupTransform(map_frame, global_frame, tf2::TimePointZero);
        static_obstacles = makeStaticCostFn(field, t.transform.translation.x, t.transform.translation.y,
                                            yawOf(t.transform.rotation));
      } catch (const std::exception & e) {
        RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "door_yield: map transform failed, no narrow point: %s", e.what());
      }
    }
  }
  const auto result = controller_->compute(robot, vel, path, people, obstacles, speed_limit_ratio,
                                      static_obstacles, people_valid);

  // 4. 全候補衝突が stuck_timeout 続いたら例外
  const auto now = clock_->now();
  if (result.all_colliding) {
    if (!colliding_) {
      colliding_ = true;
      colliding_since_ = now;
    }
    if ((now - colliding_since_).seconds() >= stuck_timeout_) {
      throw nav2_core::NoValidControl("SocialController: all candidates colliding");
    }
  } else {
    colliding_ = false;
  }

  geometry_msgs::msg::TwistStamped cmd;
  cmd.header = pose.header;
  cmd.twist.linear.x = result.cmd.vx;
  cmd.twist.linear.y = result.cmd.vy;
  cmd.twist.angular.z = result.cmd.wz;
  return cmd;
}

}  // namespace social_nav_controller

PLUGINLIB_EXPORT_CLASS(social_nav_controller::SocialController, nav2_core::Controller)
