#include "LaunchAngleSolver.h"
#include "AtmosphereModel.h"
#include "DumbArtillery.h"
#include "SimulationWorld.h"
#include <algorithm>
#include <cmath>

LaunchAngleSolver::LaunchAngleSolver() {}

std::pair<double, double>
LaunchAngleSolver::calculateLaunchAngles(const LaunchConfig &config,
                                         const Eigen::Vector3d &targetPos,
                                         const Eigen::Vector3d &launchPos) {

  // Calculate azimuth (direction in XY plane)
  double azimuth_deg = calculateAzimuth(launchPos, targetPos);

  // Calculate horizontal range to target
  Eigen::Vector3d delta = targetPos - launchPos;
  m_calculatedRange = std::sqrt(delta.x() * delta.x() + delta.y() * delta.y());

  // Find optimal elevation angle
  double elevation_deg = findOptimalElevation(config, targetPos, azimuth_deg);

  return {elevation_deg, azimuth_deg};
}

double LaunchAngleSolver::calculateAzimuth(const Eigen::Vector3d &launchPos,
                                           const Eigen::Vector3d &targetPos) {
  Eigen::Vector3d delta = targetPos - launchPos;
  double azimuth_rad = std::atan2(delta.y(), delta.x());
  double azimuth_deg = azimuth_rad * 180.0 / M_PI;

  // Normalize to [0, 360)
  while (azimuth_deg < 0.0)
    azimuth_deg += 360.0;
  while (azimuth_deg >= 360.0)
    azimuth_deg -= 360.0;

  return azimuth_deg;
}

double LaunchAngleSolver::findOptimalElevation(const LaunchConfig &config,
                                               const Eigen::Vector3d &targetPos,
                                               double azimuth_deg) {
  // Use a simple search algorithm to find the best elevation
  // We'll test elevations from 5° to 85° and find the one with minimum error

  double bestElevation = 45.0;
  double minError = 1e9;

  // Coarse search: 5° increments
  for (double elev = 5.0; elev <= 85.0; elev += 5.0) {
    Eigen::Vector3d landing =
        estimateLandingPosition(config, elev, azimuth_deg);
    double error = calculateError(landing, targetPos);

    if (error < minError) {
      minError = error;
      bestElevation = elev;
    }
  }

  // Fine search: 1° increments around best candidate
  double searchStart = std::max(5.0, bestElevation - 5.0);
  double searchEnd = std::min(85.0, bestElevation + 5.0);

  for (double elev = searchStart; elev <= searchEnd; elev += 1.0) {
    Eigen::Vector3d landing =
        estimateLandingPosition(config, elev, azimuth_deg);
    double error = calculateError(landing, targetPos);

    if (error < minError) {
      minError = error;
      bestElevation = elev;
    }
  }

  // Store the minimum error for reachability checks
  m_minimumError = minError;

  return bestElevation;
}

Eigen::Vector3d
LaunchAngleSolver::estimateLandingFromState(const LaunchConfig &config,
                                            const StateVector &currentState) {

  // Use the actual SimulationWorld for accurate prediction
  SimulationWorld tempWorld;

  // Create DumbArtillery with same config
  auto projectile = std::make_unique<DumbArtillery>(config);

  // Set the specific state (mid-flight)
  projectile->setState(currentState);

  tempWorld.addProjectile(std::move(projectile));

  // Run simulation until landing
  const double dt = 0.05; // Coarser step for performance in guidance loop
  const double max_time = 300.0;

  double t = 0.0;
  while (t < max_time) {
    tempWorld.step(dt);
    t += dt;

    const auto &projs = tempWorld.getProjectiles();
    if (projs.empty())
      break;

    if (projs[0]->hasLanded()) {
      return projs[0]->getState().position;
    }

    // Safety break if it goes underground without triggering landed (unlikely)
    if (projs[0]->getState().position.z() < -100.0) {
      return projs[0]->getState().position;
    }
  }

  // If time run out, return last position
  const auto &projs = tempWorld.getProjectiles();
  if (!projs.empty()) {
    return projs[0]->getState().position;
  }

  return Eigen::Vector3d::Zero();
}

Eigen::Vector3d LaunchAngleSolver::estimateLandingPosition(
    const LaunchConfig &config, double elevation_deg, double azimuth_deg) {

  // Use the actual SimulationWorld for accurate prediction
  // This guarantees physics match the real simulation
  SimulationWorld tempWorld;

  // Create a config copy with the specific angles
  LaunchConfig tempConfig = config;
  tempConfig.elevation_deg = elevation_deg;
  tempConfig.azimuth_deg = azimuth_deg;

  // Use DumbArtillery for ballistic prediction (ignoring guidance for now)
  auto projectile = std::make_unique<DumbArtillery>(tempConfig);
  tempWorld.addProjectile(std::move(projectile));

  // Run simulation until landing
  const double dt = 0.02; // 50Hz prediction
  const double max_time = 300.0;

  double t = 0.0;
  while (t < max_time) {
    tempWorld.step(dt);
    t += dt;

    // Check if landed
    const auto &projs = tempWorld.getProjectiles();
    if (projs.empty())
      break;

    if (projs[0]->hasLanded()) {
      return projs[0]->getState().position;
    }
  }

  // If time run out, return last position
  const auto &projs = tempWorld.getProjectiles();
  if (!projs.empty()) {
    return projs[0]->getState().position;
  }

  return Eigen::Vector3d::Zero();
}

double LaunchAngleSolver::calculateError(const Eigen::Vector3d &landingPos,
                                         const Eigen::Vector3d &targetPos) {
  // 3D Euclidean distance
  return (landingPos - targetPos).norm();
}

bool LaunchAngleSolver::isTargetReachable(const LaunchConfig &config,
                                          const Eigen::Vector3d &targetPos) {
  // Use a copy to avoid modifying the original config
  LaunchConfig configCopy = config;
  calculateLaunchAngles(configCopy, targetPos, Eigen::Vector3d::Zero());

  // Consider target reachable if the best trajectory gets within 20% of the
  // range This accounts for guidance system corrections
  double acceptableError = m_calculatedRange * 0.2;

  return m_minimumError < acceptableError;
}
