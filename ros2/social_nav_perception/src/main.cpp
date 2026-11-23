#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "social_nav_perception/people_tracker_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<social_nav_perception::PeopleTrackerNode>());
  rclcpp::shutdown();
  return 0;
}
