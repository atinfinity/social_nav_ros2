#pragma once
#include <cstddef>

#include "social_nav_core/types.hpp"
#include "social_nav_msgs/msg/tracked_people.hpp"
#include "social_nav_msgs/msg/tracked_person.hpp"

namespace social_nav_ros_common {

inline double toSeconds(const builtin_interfaces::msg::Time& t)
{
  return static_cast<double>(t.sec) + 1e-9 * static_cast<double>(t.nanosec);
}

inline social_nav::Agent toCore(const social_nav_msgs::msg::TrackedPerson& m)
{
  social_nav::Agent a;
  a.id = m.id;
  a.pos = {m.position.x, m.position.y};
  a.vel = {m.velocity.x, m.velocity.y};
  // position_covariance: xx, xy, yx, yy
  a.pos_cov = {m.position_covariance[0], 0.5 * (m.position_covariance[1] + m.position_covariance[2]),
               m.position_covariance[3]};
  a.radius = m.radius;
  a.confidence = m.confidence;
  return a;
}

inline social_nav_msgs::msg::TrackedPerson toMsg(const social_nav::Agent& a)
{
  social_nav_msgs::msg::TrackedPerson m;
  m.id = a.id;
  m.position.x = a.pos.x;
  m.position.y = a.pos.y;
  m.position.z = 0.0;
  m.velocity.x = a.vel.x;
  m.velocity.y = a.vel.y;
  m.velocity.z = 0.0;
  m.position_covariance = {a.pos_cov.xx, a.pos_cov.xy, a.pos_cov.xy, a.pos_cov.yy};
  m.radius = static_cast<float>(a.radius);
  m.confidence = static_cast<float>(a.confidence);
  return m;
}

inline social_nav::TrackedPeople toCore(const social_nav_msgs::msg::TrackedPeople& m)
{
  social_nav::TrackedPeople p;
  p.stamp = toSeconds(m.header.stamp);
  p.frame_id = m.header.frame_id;
  p.agents.reserve(m.people.size());
  for (const auto& person : m.people) {
    p.agents.push_back(toCore(person));
  }
  return p;
}

inline social_nav_msgs::msg::TrackedPeople toMsg(const social_nav::TrackedPeople& p)
{
  social_nav_msgs::msg::TrackedPeople m;
  m.header.frame_id = p.frame_id;
  const auto sec = static_cast<int32_t>(p.stamp);
  m.header.stamp.sec = sec;
  m.header.stamp.nanosec = static_cast<uint32_t>((p.stamp - sec) * 1e9);
  m.people.reserve(p.agents.size());
  for (const auto& a : p.agents) {
    m.people.push_back(toMsg(a));
  }
  return m;
}

}  // namespace social_nav_ros_common
