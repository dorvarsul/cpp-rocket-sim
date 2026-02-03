#pragma once
#include "StateVector.h"
#include <vector>

class IProjectile {
public:
  virtual ~IProjectile() = default;

  // Update the state of the projectile by a time step dt
  virtual void update(double dt) = 0;

  // Compute total forces acting on the projectile
  virtual Eigen::Vector3d
  computeForces(const StateVector &state,
                const Eigen::Vector3d &windVelocity) const = 0;

  // Get mass flow rate at a given time (kg/s)
  virtual double getMassFlowRate(double time) const = 0;

  // Update telemetry (e.g. G-load) based on current acceleration
  virtual void updateTelemetry(const Eigen::Vector3d &acceleration) {}

  // Set the current state (used by integrator)
  virtual void setState(const StateVector &state) = 0;

  // Get the current state
  virtual StateVector getState() const = 0;

  // Get the full history of states for visualization
  virtual const std::vector<StateVector> &getHistory() const = 0;

  // Check if the projectile has landed (z <= 0)
  virtual bool hasLanded() const = 0;
};
