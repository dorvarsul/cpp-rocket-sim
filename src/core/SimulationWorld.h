#pragma once
#include "DumbArtillery.h" // Needed for casting to access setState, or we should extend interface
#include "IProjectile.h"
#include <memory>
#include <vector>

class SimulationWorld {
public:
  SimulationWorld();

  void addProjectile(std::unique_ptr<IProjectile> projectile);
  void step(double dt);
  void clear();

  // Constant gravity for Sprint 1
  static constexpr double GRAVITY = 9.81;

  const std::vector<std::unique_ptr<IProjectile>> &getProjectiles() const;

private:
  std::vector<std::unique_ptr<IProjectile>> m_projectiles;

  // Derivative function for RK4: f(t, state) -> dydt
  // Sprint 2: Needs projectile reference for drag/thrust forces
  StateVector derivative(const StateVector &state, IProjectile *proj) const;
};
