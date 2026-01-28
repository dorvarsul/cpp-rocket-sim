#pragma once
#include "LaunchConfig.h"
#include "StateVector.h"
#include <Eigen/Dense>
#include <utility>

/**
 * LaunchAngleSolver
 * Calculates optimal launch angles (elevation and azimuth) to reach a target
 * position given a launch configuration.
 *
 * Uses a simplified ballistic trajectory estimation for performance.
 * For SmartArtillery, the guidance system will refine the trajectory in-flight.
 */
class LaunchAngleSolver {
public:
  LaunchAngleSolver();

  /**
   * Calculate launch angles to reach a target position
   * @param config Launch configuration (thrust, mass, etc.)
   * @param targetPos Target position in world coordinates (m)
   * @param launchPos Launch position in world coordinates (m), default is
   * origin
   * @return std::pair<elevation_deg, azimuth_deg>
   */
  std::pair<double, double> calculateLaunchAngles(
      const LaunchConfig &config, const Eigen::Vector3d &targetPos,
      const Eigen::Vector3d &launchPos = Eigen::Vector3d::Zero());

  /**
   * Estimate the landing position for given launch angles
   * Uses simplified physics for quick estimation
   * @param config Launch configuration
   * @param elevation_deg Launch elevation angle in degrees
   * @param azimuth_deg Launch azimuth angle in degrees
   * @return Estimated landing position
   */
  Eigen::Vector3d estimateLandingPosition(const LaunchConfig &config,
                                          double elevation_deg,
                                          double azimuth_deg);

  /**
   * Estimate landing position starting from a specific state
   * Used for mid-course guidance prediction
   * @param config Launch configuration (for vehicle properties)
   * @param currentState Initial state for prediction
   * @return Estimated landing position
   */
  Eigen::Vector3d estimateLandingFromState(const LaunchConfig &config,
                                           const StateVector &currentState);

  /**
   * Check if a target is reachable with given configuration
   * @param config Launch configuration
   * @param targetPos Target position
   * @return true if target can be reached, false otherwise
   */
  bool isTargetReachable(const LaunchConfig &config,
                         const Eigen::Vector3d &targetPos);

  /**
   * Get the calculated range to target (horizontal distance)
   */
  double getCalculatedRange() const { return m_calculatedRange; }

  /**
   * Get the minimum error achieved (distance to target from best trajectory)
   */
  double getMinimumError() const { return m_minimumError; }

private:
  double m_calculatedRange = 0.0;
  double m_minimumError = 0.0;

  /**
   * Helper: Calculate azimuth angle from launch to target
   */
  double calculateAzimuth(const Eigen::Vector3d &launchPos,
                          const Eigen::Vector3d &targetPos);

  /**
   * Helper: Find optimal elevation angle using optimization
   */
  double findOptimalElevation(const LaunchConfig &config,
                              const Eigen::Vector3d &targetPos,
                              double azimuth_deg);

  /**
   * Helper: Calculate error between estimated landing and target
   */
  double calculateError(const Eigen::Vector3d &landingPos,
                        const Eigen::Vector3d &targetPos);
};
