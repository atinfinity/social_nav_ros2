#pragma once
// sim の安全層のモード。cm / cm_stop は ROS の collision_monitor(PolygonStop・PolygonSlow)の写し。
#include <string>

#include "social_nav_core/types.hpp"

namespace social_nav::sim {

enum class SafetyMode { Predictive, CmStop, Cm, Off };

constexpr double kCmStopClearance = 0.15;   // yaml: 停止円 = 外形 + 0.15 m
constexpr double kCmSlowClearance = 0.5;    // yaml: 減速円 = 外形 + 0.5 m
constexpr double kCmSlowRatio = 0.4;        // yaml: slowdown_ratio

inline bool parseSafetyMode(const std::string& s, SafetyMode& m) {
  if (s == "predictive") m = SafetyMode::Predictive;
  else if (s == "cm_stop") m = SafetyMode::CmStop;
  else if (s == "cm") m = SafetyMode::Cm;
  else if (s == "off") m = SafetyMode::Off;
  else return false;
  return true;
}

struct CmOutcome {
  bool stop{false};
  double scale{1.0};   // 指令に掛ける倍率(stop のときは使わない)
};

// min_clearance = 現在の外形間距離の最小(人の半径を引いた値)。向きも縮みも見ない(CM は静的な円)
inline CmOutcome cmSafety(SafetyMode m, double min_clearance) {
  CmOutcome o;
  if (m != SafetyMode::CmStop && m != SafetyMode::Cm) return o;
  if (min_clearance < kCmStopClearance) { o.stop = true; return o; }
  if (m == SafetyMode::Cm && min_clearance < kCmSlowClearance) o.scale = kCmSlowRatio;
  return o;
}

}  // namespace social_nav::sim
