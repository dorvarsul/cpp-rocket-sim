#pragma once
#include "AtmosphereModel.h"
#include <Eigen/Dense>

/**
 * Aerodynamic Component
 * Computes drag forces with Mach-dependent drag coefficient
 */
class AeroComponent {
public:
  /**
   * Constructor
   * @param referenceArea_m2 Cross-sectional area in m²
   * @param atm Reference to atmosphere model
   */
  AeroComponent(double referenceArea_m2, const AtmosphereModel &atm);

  /**
   * Compute drag force vector
   * @param velocity Current velocity vector (m/s)
   * @param altitude_m Current altitude (m)
   * @return Drag force vector in Newtons (opposite to velocity direction)
   */
  Eigen::Vector3d computeDragForce(const Eigen::Vector3d &velocity,
                                   double altitude_m) const;

  /**
   * Calculate Mach number
   * @param velocity Current velocity vector (m/s)
   * @param altitude_m Current altitude (m)
   * @return Mach number (dimensionless)
   */
  double getMachNumber(const Eigen::Vector3d &velocity,
                       double altitude_m) const;

  /**
   * Get drag coefficient based on Mach number
   * @param machNumber Current Mach number
   * @return Drag coefficient (dimensionless)
   */
  double getDragCoefficient(double machNumber) const;

  /**
   * Get reference area (for fin lift calculations)
   * @return Reference area in m²
   */
  double getReferenceArea() const { return m_referenceArea; }

private:
  double m_referenceArea; // Cross-sectional area (m²)
  const AtmosphereModel &m_atmosphere;
};
