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
  // Deprecated for direct calls, better used via computeThrustForce with
  // override But for compatibility:
  return computeThrustForce(0, velocity).normalized(); // Hacky
}

Eigen::Vector3d PropulsionSystem::computeThrustForce(
    double elapsedTime_s, const Eigen::Vector3d &velocity,
    const Eigen::Vector3d &launchRailDirection) const {
  double thrustMagnitude = getThrust(elapsedTime_s);
  if (thrustMagnitude < 1e-6) {
    return Eigen::Vector3d::Zero();
  }

  double speed = velocity.norm();

  // Launch Rail Logic:
  // If speed is low (< 20 m/s) OR we explicitly provided a rail direction and
  // time is small (< 0.5s) we force thrust to preserve launch orientation. In a
  // 3-DOF sim, orientation = velocity direction. At Speed=0, velocity direction
  // is undefined/zero. Gravity pulls velocity 'down' immediately. Real rockets
  // have a rail that constrains them.

  // Stability Threshold (m/s)
  const double STABLE_SPEED = 20.0;

  Eigen::Vector3d direction;

  if (speed < STABLE_SPEED) {
    // Unstable / On Rail
    // If caller provided a specific direction (DumbArtillery/SmartArtillery
    // should), use it.
    if (launchRailDirection.norm() > 0.1) {
      direction = launchRailDirection.normalized();
    } else {
      // Fallback to internal profile default
      direction = m_profile.launchDirection;
    }
  } else {
    // Aerodynamically Stable flight
    direction = velocity.normalized();
  }

  return thrustMagnitude * direction;
}
