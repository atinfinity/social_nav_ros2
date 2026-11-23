#include "social_nav_bt_nodes/hold_position.hpp"

namespace social_nav_bt
{

HoldPosition::HoldPosition(const std::string & name, const NodeConfig & config)
: BT::StatefulActionNode(name, config)
{
}

BT::PortsList HoldPosition::providedPorts()
{
  return {BT::InputPort<std::string>("behavior", "PROCEED", "PROCEED/FOLLOW/YIELD/WAIT")};
}

BT::NodeStatus HoldPosition::check()
{
  auto behavior = getInput<std::string>("behavior");
  if (!behavior) {
    return BT::NodeStatus::FAILURE;
  }
  const std::string & b = behavior.value();
  return (b == "YIELD" || b == "WAIT") ? BT::NodeStatus::RUNNING : BT::NodeStatus::FAILURE;
}

BT::NodeStatus HoldPosition::onStart() {return check();}

BT::NodeStatus HoldPosition::onRunning() {return check();}

void HoldPosition::onHalted() {}

}  // namespace social_nav_bt
