#include "social_nav_bt_nodes/bt_compat.hpp"
#include "social_nav_bt_nodes/hold_position.hpp"
#include "social_nav_bt_nodes/select_social_behavior.hpp"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<social_nav_bt::SelectSocialBehavior>("SelectSocialBehavior");
  factory.registerNodeType<social_nav_bt::HoldPosition>("HoldPosition");
}
