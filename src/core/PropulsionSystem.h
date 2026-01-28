#pragma once
#include <Eigen/Dense>

/**
 * Thrust Profile Configuration
 */
struct ThrustProfile {
  double thrustNewtons;            // Thrust magnitude (N)
  double burnDuration_s;           // Burn duration (s)
  double massFlowRate_kgps;        // Fuel consumption rate (kg/s)
  Eigen::Vector3d launchDirection; // Unit vector for initial thrust direction
};

/**
 * Propulsion System
 * Manages thrust generation and fuel consumption
 */
class PropulsionSystem {
public:
  /**
   * Constructor
   * @param profile Thrust profile configuration
   */
  PropulsionSystem(const ThrustProfile &profile);

  /**
   * Get current thrust magnitude
   * @param elapsedTime_s Time since launch (s)
   * @return Thrust force in Newtons
   */
  double getThrust(double elapsedTime_s) const;

  /**
   * Get total fuel consumed up to given time
   * @param elapsedTime_s Time since launch (s)
   * @return Fuel consumed in kg
   */
  double getFuelConsumed(double elapsedTime_s) const;

  /**
   * Check if engine is currently burning
   * @param elapsedTime_s Time since launch (s)
   * @return True if burning, false otherwise
   */
  bool isBurning(double elapsedTime_s) const;

  /**
   * Get thrust vector direction (along velocity or vertical if at rest)
   * @param velocity Current velocity vector
   * @return Unit vector in thrust direction
   */
  Eigen::Vector3d getThrustDirection(const Eigen::Vector3d &velocity) const;

  /**
   * Compute thrust force vector
   * @param elapsedTime_s Time since launch (s)
   * @param velocity Current velocity vector
   * @return Thrust force vector in Newtons
   */
  Eigen::Vector3d computeThrustForce(double elapsedTime_s,
                                     const Eigen::Vector3d &velocity) const;

private:
  ThrustProfile m_profile;
};
