#include "ControlSystem.h"
#include <algorithm>
#include <cmath>

ControlSystem::ControlSystem(const ControlLimits &limits) : m_limits(limits) {}

Eigen::Vector2d
ControlSystem::computeFinDeflection(const Eigen::Vector3d &guidanceAccel,
                                    const Eigen::Vector3d &velocity,
                                    double dynamicPressure) {
  Eigen::Vector2d finAngles = Eigen::Vector2d::Zero();

  double speed = velocity.norm();
  if (speed < 1.0 || dynamicPressure < 0.1) {
    // Too slow for aerodynamic control
    m_currentFinAngles = finAngles;
    return finAngles;
  }

  // Velocity direction (body frame approximation)
  Eigen::Vector3d velDir = velocity.normalized();

  // Decompose guidance acceleration into perpendicular components
  // Pitch control (up/down in velocity frame)
  Eigen::Vector3d up(0, 0, 1);
  Eigen::Vector3d right = velDir.cross(up).normalized();
  if (right.norm() < 0.1) {
    // Nearly vertical, use different reference
    right = Eigen::Vector3d(1, 0, 0);
  }
  Eigen::Vector3d pitchAxis = right.cross(velDir).normalized();

  double pitchAccel = guidanceAccel.dot(pitchAxis);
  double yawAccel = guidanceAccel.dot(right);

  // Convert accelerations to fin angles
  // Lift = 0.5 * rho * v^2 * S * CL
  // CL = CL_alpha * alpha
  // a = L / m, solve for alpha

  // Simplified: assume reference area and mass give ~1:1 mapping
  // This is a placeholder - proper implementation needs mass and area
  double area = 0.02;           // Approximate fin area
  double mass_estimate = 100.0; // Rough estimate

  if (dynamicPressure > 0.1) {
    double liftFactor =
        0.5 * dynamicPressure * area * m_liftSlope / mass_estimate;

    if (liftFactor > 0.01) {
      finAngles(0) = pitchAccel / liftFactor; // Pitch fin angle
      finAngles(1) = yawAccel / liftFactor;   // Yaw fin angle
    }
  }

  // Apply limits
  double maxAngle_rad = m_limits.maxFinDeflection_deg * M_PI / 180.0;
  finAngles(0) = std::clamp(finAngles(0), -maxAngle_rad, maxAngle_rad);
  finAngles(1) = std::clamp(finAngles(1), -maxAngle_rad, maxAngle_rad);

  m_currentFinAngles = finAngles;
  return finAngles;
}

Eigen::Vector3d ControlSystem::computeFinLift(const Eigen::Vector2d &finAngles,
                                              const Eigen::Vector3d &velocity,
                                              double dynamicPressure,
                                              double referenceArea) {
  double speed = velocity.norm();
  if (speed < 1.0) {
    return Eigen::Vector3d::Zero();
  }

  Eigen::Vector3d velDir = velocity.normalized();

  // Build perpendicular axes
  Eigen::Vector3d up(0, 0, 1);
  Eigen::Vector3d right = velDir.cross(up).normalized();
  if (right.norm() < 0.1) {
    right = Eigen::Vector3d(1, 0, 0);
  }
  Eigen::Vector3d pitchAxis = right.cross(velDir).normalized();

  // Lift from pitch fins (perpendicular to velocity in pitch plane)
  double liftCoeff_pitch = m_liftSlope * finAngles(0);
  double liftMag_pitch =
      0.5 * dynamicPressure * referenceArea * liftCoeff_pitch;
  Eigen::Vector3d liftForce_pitch = liftMag_pitch * pitchAxis;

  // Lift from yaw fins (perpendicular to velocity in yaw plane)
  double liftCoeff_yaw = m_liftSlope * finAngles(1);
  double liftMag_yaw = 0.5 * dynamicPressure * referenceArea * liftCoeff_yaw;
  Eigen::Vector3d liftForce_yaw = liftMag_yaw * right;

  return liftForce_pitch + liftForce_yaw;
}

double ControlSystem::getCurrentGLoad(const Eigen::Vector3d &totalAccel) const {
  // G-load is magnitude of total acceleration divided by g
  const double g = 9.81;
  return totalAccel.norm() / g;
}
