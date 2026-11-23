#pragma once
#include <string>

#include "social_nav_bt_nodes/bt_compat.hpp"

namespace social_nav_bt
{

// behavior が YIELD / WAIT の間 RUNNING、それ以外は FAILURE。SUCCESS は返さない。判断はしない。
class HoldPosition : public BT::StatefulActionNode
{
public:
  HoldPosition(const std::string & name, const NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  BT::NodeStatus check();
};

}  // namespace social_nav_bt
