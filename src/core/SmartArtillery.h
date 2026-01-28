#pragma once
#include "ControlSystem.h"
#include "DumbArtillery.h"
#include "GuidanceUnit.h"
#include "NavigationSystem.h"

/**
 * SmartArtillery
 * Guided munition with full GNC (Guidance, Navigation, Control) capability
 * Extends DumbArtillery by adding target tracking and fin control
 */
class SmartArtillery : public DumbArtillery {
public:
  /**
   * Constructor
   * @param config Launch configuration
   * @param guidanceConfig Guidance system configuration
   * @param controlLimits Control system limits
   * @param sensorNoiseEnabled Enable sensor noise simulation
   */
  SmartArtillery(const LaunchConfig &config,
                 const GuidanceConfig &guidanceConfig,
                 const ControlLimits &controlLimits,
                 bool sensorNoiseEnabled = false);

  /**
   * Get distance to target (slant range)
   */
  double getDistanceToTarget() const;

  /**
   * Get current G-load
   */
  double getCurrentGLoad() const { return m_currentGLoad; }

  /**
   * Check if target has been reached
   */
  bool hasReachedTarget() const;

  /**
   * Get current fin deflection angles (for telemetry)
   */
  Eigen::Vector2d getFinAngles() const;

  /**
   * Enable/disable guidance (for testing dumb vs smart)
   */
  void setGuidanceEnabled(bool enabled) { m_guidanceEnabled = enabled; }
  bool isGuidanceEnabled() const { return m_guidanceEnabled; }

  /**
   * Check if guidance is currently active (enabled AND above min velocity)
   */
  bool isGuidanceActive() const {
    const double MIN_GUIDANCE_VELOCITY = 50.0;
    return m_guidanceEnabled && !m_landed &&
           m_state.velocity.norm() >= MIN_GUIDANCE_VELOCITY;
  }

  /**
   * Update target position (for moving targets)
   */
  void setTarget(const Eigen::Vector3d &newTarget);

  /**
   * Compute fin lift force for current state (for SimulationWorld physics)
   * @param state Current state vector
   * @param windVelocity Current wind velocity vector (default zero)
   * @return Fin-generated lift force in Newtons
   */
  Eigen::Vector3d
  computeFinLift(const StateVector &state,
                 const Eigen::Vector3d &windVelocity = Eigen::Vector3d::Zero());

  /**
   * Update current G-load (called by SimulationWorld after computing total
   * acceleration)
   * @param totalAcceleration Total acceleration vector including all forces
   */
  void setCurrentGLoad(const Eigen::Vector3d &totalAcceleration);

  /**
   * Override to add guidance forces
   */
  void update(double dt) override;

private:
  // GNC Components
  NavigationSystem m_navigation;
  GuidanceUnit m_guidance;
  ControlSystem m_control;

  bool m_guidanceEnabled = true;
  double m_currentGLoad = 0.0;

  /**
   * Compute total forces including guidance
   */
  Eigen::Vector3d computeTotalForces(double elapsedTime);
};
