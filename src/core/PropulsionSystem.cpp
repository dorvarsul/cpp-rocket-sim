#include "PropulsionSystem.h"
#include <algorithm>

PropulsionSystem::PropulsionSystem(const ThrustProfile &profile)
    : m_profile(profile) {}

bool PropulsionSystem::isBurning(double elapsedTime_s) const {
  return elapsedTime_s >= 0.0 && elapsedTime_s < m_profile.burnDuration_s;
}

double PropulsionSystem::getThrust(double elapsedTime_s) const {
  if (isBurning(elapsedTime_s)) {
    return m_profile.thrustNewtons;
  }
  return 0.0;
}

double PropulsionSystem::getFuelConsumed(double elapsedTime_s) const {
  if (elapsedTime_s <= 0.0) {
    return 0.0;
  }

  double burnTime = std::min(elapsedTime_s, m_profile.burnDuration_s);
  return m_profile.massFlowRate_kgps * burnTime;
}

Eigen::Vector3d
PropulsionSystem::getThrustDirection(const Eigen::Vector3d &velocity) const {
  double speed = velocity.norm();

  // If moving, thrust along velocity direction
  if (speed > 1e-6) {
    return velocity / speed;
  }

  // If at rest, thrust in launch direction
  return m_profile.launchDirection;
}

Eigen::Vector3d
PropulsionSystem::computeThrustForce(double elapsedTime_s,
                                     const Eigen::Vector3d &velocity) const {
  double thrustMagnitude = getThrust(elapsedTime_s);
  if (thrustMagnitude < 1e-6) {
    return Eigen::Vector3d::Zero();
  }

  Eigen::Vector3d direction = getThrustDirection(velocity);
  return thrustMagnitude * direction;
}
