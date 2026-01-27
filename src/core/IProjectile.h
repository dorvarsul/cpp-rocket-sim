#pragma once
#include "StateVector.h"
#include <vector>

class IProjectile {
public:
  virtual ~IProjectile() = default;

  // Update the state of the projectile by a time step dt
  virtual void update(double dt) = 0;

  // Get the current state
  virtual StateVector getState() const = 0;

  // Get the full history of states for visualization
  virtual std::vector<StateVector> getHistory() const = 0;

  // Check if the projectile has landed (z <= 0)
  virtual bool hasLanded() const = 0;
};
