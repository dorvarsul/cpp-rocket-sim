#pragma once
#include "AeroComponent.h"
#include "AtmosphereModel.h"
#include "IProjectile.h"
#include "LaunchConfig.h"
#include "MassProperties.h"
#include "PropulsionSystem.h"
#include <vector>

class DumbArtillery : public IProjectile {
public:
  explicit DumbArtillery(const LaunchConfig &config);

  // Compute total forces acting on the projectile
  Eigen::Vector3d
  computeForces(const StateVector &state,
                const Eigen::Vector3d &windVelocity) const override;

  void update(double dt) override;
  StateVector getState() const override;
  const std::vector<StateVector> &getHistory() const override;
  bool hasLanded() const override;

  // Get mass flow rate at a given time (kg/s)
  double getMassFlowRate(double time) const override;

  // Helper for RK4
  void setState(const StateVector &newState) override;

  // Sprint 2: Accessors for telemetry and physics
  double getMachNumber() const;
  Eigen::Vector3d getDragForce() const;
  Eigen::Vector3d getThrustForce() const;
  double getCurrentMass() const;
  double getRemainingFuel() const;

  // Sprint 2: Physics component access for SimulationWorld
  const AeroComponent &getAero() const { return m_aero; }
  const PropulsionSystem &getPropulsion() const { return m_propulsion; }
  const MassProperties &getMass() const { return m_mass; }
  const AtmosphereModel &getAtmosphere() const { return m_atmosphere; }

protected:
  StateVector m_state;
  std::vector<StateVector> m_history;
  bool m_landed = false;

  // Sprint 2: Physics components
  AtmosphereModel m_atmosphere;
  AeroComponent m_aero;
  PropulsionSystem m_propulsion;
  MassProperties m_mass;
  Eigen::Vector3d m_launchDirection;
};
