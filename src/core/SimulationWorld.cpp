#include "SimulationWorld.h"
#include <iostream>

SimulationWorld::SimulationWorld() {}

void SimulationWorld::addProjectile(std::unique_ptr<IProjectile> projectile) {
  m_projectiles.push_back(std::move(projectile));
}

const std::vector<std::unique_ptr<IProjectile>> &
SimulationWorld::getProjectiles() const {
  return m_projectiles;
}

void SimulationWorld::clear() { m_projectiles.clear(); }

StateVector SimulationWorld::derivative(const StateVector &state) const {
  StateVector output;
  // d(pos)/dt = velocity
  output.position = state.velocity;
  // d(vel)/dt = acceleration (gravity only for Sprint 1)
  output.velocity = Eigen::Vector3d(0.0, 0.0, -GRAVITY);

  return output;
}

void SimulationWorld::step(double dt) {
  for (auto &proj : m_projectiles) {
    if (proj->hasLanded())
      continue;

    // We need to cast to DumbArtillery to set state directly or add setState to
    // interface For the sake of OOP purity as requested, we might want to
    // expose a way to apply updates But for now, let's assume we can cast or we
    // should add `applyState` to IProjectile. Given Requirements: IProjectile
    // has update(dt), but SimulationWorld must use RK4. So SimulationWorld
    // needs to compute next state and set it. Let's dynamic_cast for now to
    // access setState.

    auto *artillery = dynamic_cast<DumbArtillery *>(proj.get());
    if (!artillery)
      continue;

    StateVector current = artillery->getState();

    // RK4 Integration
    // k1
    StateVector k1 = derivative(current);

    // k2
    StateVector k2State = current + k1 * (0.5 * dt);
    StateVector k2 = derivative(k2State);

    // k3
    StateVector k3State = current + k2 * (0.5 * dt);
    StateVector k3 = derivative(k3State);

    // k4
    StateVector k4State = current + k3 * dt;
    StateVector k4 = derivative(k4State);

    // Final state
    StateVector nextState =
        current + (k1 + k2 * 2.0 + k3 * 2.0 + k4) * (dt / 6.0);

    artillery->setState(nextState);
  }
}
