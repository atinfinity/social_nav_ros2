#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include "critic_data_adapter.hpp"
#include "nav2_mppi_controller/critic_function.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "social_nav_core/footprint.hpp"
#include "social_nav_core/trajectory_cost.hpp"
#include "social_nav_ros_common/people_subscriber.hpp"
#include "social_nav_ros_common/robot_model_params.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"

namespace mppi::critics
{

class SocialCritic : public CriticFunction
{
public:
  void initialize() override
  {
    auto node = parent_.lock();
    if (!node) {
      throw std::runtime_error("SocialCritic: parent node expired");
    }
    auto getParam = parameters_handler_->getParamGetter(name_);
    std::string people_topic;
    double stale_timeout, min_confidence;
    getParam(weight_, "cost_weight", 5.0);
    getParam(power_, "cost_power", 1);
    getParam(people_topic, "people_topic", std::string("/social_nav/tracked_people"));
    getParam(stale_timeout, "stale_timeout", 0.5);
    getParam(min_confidence, "min_confidence", 0.3);

    social_nav::SocialCostParams p;
    getParam(p.prox.sigma, "sigma", 0.45);
    getParam(p.prox.velocity_gain, "velocity_gain", 0.4);
    getParam(p.prox.rear_ratio, "rear_ratio", 0.6);
    getParam(p.collision_margin, "collision_margin", 0.1);
    getParam(p.collision_cost, "collision_cost", 1000.0);
    getParam(p.time_decay_tau, "time_decay_tau", 2.0);
    getParam(p.step_subsample, "step_subsample", 2);
    getParam(p.max_range, "max_range", 6.0);
    p.min_confidence = min_confidence;

    std::shared_ptr<const social_nav::Footprint> footprint =
      social_nav::makeFootprint(social_nav_ros_common::readFootprintSpec(node, name_));
    cost_params_ = p;
    footprint_ = footprint;

    people_ = std::make_unique<social_nav_ros_common::PeopleSubscriber>(
      node, people_topic, stale_timeout, min_confidence);
    clock_ = node->get_clock();
  }

  void score(CriticData & data) override
  {
    if (!enabled_) {
      return;
    }
    auto people = people_->get();
    if (people.agents.empty() || !toCostmapFrame(people)) {
      return;
    }
    const auto info = social_nav_mppi_critics::batchInfo(data);
    const double t_offset = std::max(0.0, clock_->now().seconds() - people.stamp);

    // prepare は非 const なので、呼び出しごとのローカルインスタンスで行う(score 自体は並列可)
    social_nav::SocialTrajectoryCost cost(cost_params_, footprint_);
    cost.prepare(people, info.robot_pos, t_offset, info.dt, info.steps);
    if (cost.empty()) {
      return;
    }
    for (std::size_t i = 0; i < info.batch; ++i) {
      const double c = cost.score(social_nav_mppi_critics::makeView(data, i));
      social_nav_mppi_critics::addCost(data, i, weight_ * std::pow(c, power_));
    }
  }

private:
  // people の frame が costmap の global_frame と違えば変換する。失敗したら false
  bool toCostmapFrame(social_nav::TrackedPeople & people)
  {
    const std::string target = costmap_ros_->getGlobalFrameID();
    if (people.frame_id.empty() || people.frame_id == target) {
      return true;
    }
    try {
      const auto tf = costmap_ros_->getTfBuffer()->lookupTransform(
        target, people.frame_id, tf2::TimePointZero);
      const double yaw = tf2::getYaw(tf.transform.rotation);
      const double c = std::cos(yaw), s = std::sin(yaw);
      const auto & tr = tf.transform.translation;
      for (auto & a : people.agents) {
        const auto pos = a.pos;
        a.pos = {c * pos.x - s * pos.y + tr.x, s * pos.x + c * pos.y + tr.y};
        const auto v = a.vel;
        a.vel = {c * v.x - s * v.y, s * v.x + c * v.y};
        const auto cv = a.pos_cov;
        a.pos_cov.xx = c * c * cv.xx - 2 * c * s * cv.xy + s * s * cv.yy;
        a.pos_cov.xy = c * s * (cv.xx - cv.yy) + (c * c - s * s) * cv.xy;
        a.pos_cov.yy = s * s * cv.xx + 2 * c * s * cv.xy + c * c * cv.yy;
      }
      people.frame_id = target;
      return true;
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "SocialCritic: TF failed: %s", e.what());
      return false;
    }
  }

  double weight_{5.0};
  int power_{1};
  social_nav::SocialCostParams cost_params_;
  std::shared_ptr<const social_nav::Footprint> footprint_;
  std::unique_ptr<social_nav_ros_common::PeopleSubscriber> people_;
  rclcpp::Clock::SharedPtr clock_;
};

}  // namespace mppi::critics

PLUGINLIB_EXPORT_CLASS(mppi::critics::SocialCritic, mppi::critics::CriticFunction)
