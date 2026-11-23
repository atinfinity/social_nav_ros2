#pragma once
// BT.CPP v3(Humble)/ v4(Jazzy 以降)の差を局所化する。#if はこのファイルだけに置く。
// ノード本体は v3/v4 共通の API(providedPorts, getInput, setOutput, onStart/onRunning/onHalted)だけを使う。
#if __has_include(<behaviortree_cpp/bt_factory.h>)
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
namespace social_nav_bt
{
using NodeConfig = BT::NodeConfig;
}
#else
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
namespace social_nav_bt
{
using NodeConfig = BT::NodeConfiguration;
}
#endif
