#pragma once
#include <cctype>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/parameter.hpp"
#include "social_nav_core/footprint.hpp"

namespace social_nav_ros_common {

// "[[x1,y1],[x2,y2],...]"(Nav2 の footprint と同じ書式)を解釈する。不正なら std::invalid_argument
inline std::vector<social_nav::Vec2> parseFootprintString(const std::string & s)
{
  std::vector<double> nums;
  const char * p = s.c_str();
  const char * end = p + s.size();
  while (p < end) {
    if (std::isdigit(static_cast<unsigned char>(*p)) || *p == '-' || *p == '+' || *p == '.') {
      char * next = nullptr;
      nums.push_back(std::strtod(p, &next));
      if (next == p) {
        throw std::invalid_argument("invalid footprint string: " + s);
      }
      p = next;
    } else if (*p == '[' || *p == ']' || *p == ',' || std::isspace(static_cast<unsigned char>(*p))) {
      ++p;
    } else {
      throw std::invalid_argument("invalid footprint string: " + s);
    }
  }
  if (nums.size() % 2 != 0) {
    throw std::invalid_argument("footprint needs x,y pairs: " + s);
  }
  std::vector<social_nav::Vec2> pts;
  for (size_t i = 0; i < nums.size(); i += 2) {
    pts.push_back({nums[i], nums[i + 1]});
  }
  return pts;
}

namespace detail {
template<class NodePtr, class T>
T readParam(const NodePtr & node, const std::string & name, const T & def)
{
  if (!node->has_parameter(name)) {
    node->declare_parameter(name, rclcpp::ParameterValue(def));
  }
  T value = def;
  node->get_parameter(name, value);
  return value;
}
}  // namespace detail

// <prefix>.footprint_type(circle) / <prefix>.robot_radius(0.3) / <prefix>.footprint("[]")
template<class NodePtr>
social_nav::FootprintSpec readFootprintSpec(const NodePtr & node, const std::string & prefix)
{
  social_nav::FootprintSpec spec;
  spec.type = detail::readParam<NodePtr, std::string>(node, prefix + ".footprint_type", "circle");
  spec.radius = detail::readParam<NodePtr, double>(node, prefix + ".robot_radius", 0.3);
  const auto text = detail::readParam<NodePtr, std::string>(node, prefix + ".footprint", "[]");
  spec.points = parseFootprintString(text);
  return spec;
}

// <prefix>.motion_model(diff_drive)
template<class NodePtr>
std::string readMotionModel(const NodePtr & node, const std::string & prefix)
{
  return detail::readParam<NodePtr, std::string>(node, prefix + ".motion_model", "diff_drive");
}

}  // namespace social_nav_ros_common
