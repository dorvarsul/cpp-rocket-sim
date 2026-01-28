#pragma once
#include "StateVector.h"
#include <random>

/**
 * NavigationSystem
 * Simulates GPS/IMU sensor providing estimated state with optional noise
 */
class NavigationSystem {
public:
  NavigationSystem();

  /**
   * Get estimated state with optional sensor noise
   * @param trueState The actual state from physics simulation
   * @return Estimated state with sensor errors applied
   */
  StateVector getEstimatedState(const StateVector &trueState);

  /**
   * Enable/disable sensor noise simulation
   * @param enabled True to add GPS/IMU inaccuracies
   */
  void setSensorNoise(bool enabled);

  /**
   * Check if sensor noise is enabled
   */
  bool isSensorNoiseEnabled() const { return m_sensorNoiseEnabled; }

  /**
   * Configure noise parameters
   */
  void setPositionNoise(double noise_m) { m_positionNoise_m = noise_m; }
  void setVelocityNoise(double noise_mps) { m_velocityNoise_mps = noise_mps; }

private:
  bool m_sensorNoiseEnabled = false;
  double m_positionNoise_m = 5.0;   // GPS accuracy (~5m typical)
  double m_velocityNoise_mps = 0.5; // IMU drift

  // Random number generation for noise
  std::mt19937 m_rng;
  std::normal_distribution<double> m_normalDist;
};
