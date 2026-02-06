#pragma once

#include "GuidanceConfig.h"
#include "LaunchAngleSolver.h"
#include "LaunchConfig.h"
#include "StateVector.h"
#include <Eigen/Dense>

/**
 * Guidance Unit
 * Implements guidance laws (Proportional Navigation, Predictive Guidance)
 */
class GuidanceUnit {
public:
  GuidanceUnit(const GuidanceConfig &config, const LaunchConfig &launchConfig);

  /**
   * Compute guidance acceleration command
   * @param state Current vehicle state
   * @return Desired lateral acceleration vector (m/s^2)
   */
  Eigen::Vector3d computeGuidanceCommand(const StateVector &state);

  // Getters for telemetry
  double getDistanceToTarget(const Eigen::Vector3d &position) const;
  bool hasReachedTarget(const Eigen::Vector3d &position) const;
  void setTarget(const Eigen::Vector3d &newTarget);

private:
  GuidanceConfig m_config;
  LaunchConfig m_launchConfig;
  LaunchAngleSolver m_predictor; // For predictive guidance

  Eigen::Vector3d m_lastLOS;

  // Cache guidance command for low-frequency updates
  Eigen::Vector3d m_cachedGuidanceCmd = Eigen::Vector3d::Zero();
  double m_lastGuidanceUpdateTime = -1.0;

  // Predictive Guidance Implementation
  Eigen::Vector3d computePredictiveGuidance(const StateVector &state);
  Eigen::Vector3d computeProNavGuidance(const StateVector &state);
};
