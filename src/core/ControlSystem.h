#pragma once
#include <Eigen/Dense>

/**
 * ControlLimits
 * Physical limits for control system
 */
struct ControlLimits {
  double maxFinDeflection_deg = 20.0; // Maximum fin angle
  double maxGLoad = 10.0;             // Structural G-limit
};

/**
 * ControlSystem
 * Translates guidance commands into fin deflections and aerodynamic forces
 */
class ControlSystem {
public:
  ControlSystem(const ControlLimits &limits);

  /**
   * Compute required fin deflection angles from guidance command
   * @param guidanceAccel Desired lateral acceleration from guidance
   * @param velocity Current velocity vector
   * @param dynamicPressure q = 0.5 * rho * v^2
   * @return Fin angles (pitch, yaw) in radians
   */
  Eigen::Vector2d computeFinDeflection(const Eigen::Vector3d &guidanceAccel,
                                       const Eigen::Vector3d &velocity,
                                       double dynamicPressure);

  /**
   * Calculate aerodynamic lift force from fin deflections
   * @param finAngles Fin deflection (pitch, yaw) in radians
   * @param velocity Current velocity vector
   * @param dynamicPressure q = 0.5 * rho * v^2
   * @param referenceArea Cross-sectional area
   * @return Lift force vector in world frame
   */
  Eigen::Vector3d computeFinLift(const Eigen::Vector2d &finAngles,
                                 const Eigen::Vector3d &velocity,
                                 double dynamicPressure, double referenceArea);

  /**
   * Calculate current G-load from total acceleration
   * @param totalAccel Net acceleration including gravity
   * @return G-load magnitude (1.0 = 1G)
   */
  double getCurrentGLoad(const Eigen::Vector3d &totalAccel) const;

  /**
   * Get current fin angles (for telemetry)
   */
  Eigen::Vector2d getFinAngles() const { return m_currentFinAngles; }

private:
  ControlLimits m_limits;
  Eigen::Vector2d m_currentFinAngles =
      Eigen::Vector2d::Zero(); // Current fin state

  // Aerodynamic coefficients (simplified)
  double m_liftSlope = 3.5; // CL_alpha in per radian (typical for fins)
};
