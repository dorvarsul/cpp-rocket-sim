#include "AeroComponent.h"
#include <cmath>

AeroComponent::AeroComponent(double referenceArea_m2,
                             const AtmosphereModel &atm)
    : m_referenceArea(referenceArea_m2), m_atmosphere(atm) {}

double AeroComponent::getMachNumber(const Eigen::Vector3d &velocity,
                                    double altitude_m) const {
  double speed = velocity.norm();
  double speedOfSound = m_atmosphere.getSpeedOfSound(altitude_m);
  return speed / speedOfSound;
}

double AeroComponent::getDragCoefficient(double machNumber) const {
  // Mach-dependent drag coefficient lookup
  if (machNumber < 0.8) {
    // Subsonic
    return 0.15;
  } else if (machNumber < 1.2) {
    // Transonic (drag rise near sound barrier)
    return 0.35;
  } else {
    // Supersonic
    return 0.25;
  }
}

Eigen::Vector3d AeroComponent::computeDragForce(const Eigen::Vector3d &velocity,
                                                double altitude_m) const {
  double speed = velocity.norm();

  // No drag if not moving
  if (speed < 1e-6) {
    return Eigen::Vector3d::Zero();
  }

  // Get atmospheric density
  double rho = m_atmosphere.getDensity(altitude_m);

  // Get Mach number and drag coefficient
  double mach = getMachNumber(velocity, altitude_m);
  double Cd = getDragCoefficient(mach);

  // Drag equation: F_d = 0.5 * ρ * v² * C_d * A
  double dragMagnitude = 0.5 * rho * speed * speed * Cd * m_referenceArea;

  // Drag opposes velocity direction
  Eigen::Vector3d velocityDirection = velocity / speed;
  return -dragMagnitude * velocityDirection;
}
