#include "NavigationSystem.h"
#include <random>

NavigationSystem::NavigationSystem()
    : m_rng(std::random_device{}()), m_normalDist(0.0, 1.0) {}

StateVector NavigationSystem::getEstimatedState(const StateVector &trueState) {
  if (!m_sensorNoiseEnabled) {
    return trueState; // Perfect sensor
  }

  // Add Gaussian noise to position and velocity
  StateVector estimated = trueState;

  // Position noise (GPS error)
  estimated.position.x() += m_normalDist(m_rng) * m_positionNoise_m;
  estimated.position.y() += m_normalDist(m_rng) * m_positionNoise_m;
  estimated.position.z() += m_normalDist(m_rng) * m_positionNoise_m;

  // Velocity noise (IMU drift)
  estimated.velocity.x() += m_normalDist(m_rng) * m_velocityNoise_mps;
  estimated.velocity.y() += m_normalDist(m_rng) * m_velocityNoise_mps;
  estimated.velocity.z() += m_normalDist(m_rng) * m_velocityNoise_mps;

  return estimated;
}

void NavigationSystem::setSensorNoise(bool enabled) {
  m_sensorNoiseEnabled = enabled;
}
