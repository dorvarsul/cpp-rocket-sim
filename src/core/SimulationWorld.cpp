#include "SimulationWorld.h"
#include "SmartArtillery.h"
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

StateVector SimulationWorld::derivative(const StateVector &state,
                                        IProjectile *proj) const {
  StateVector output;

  // d(pos)/dt = velocity
  output.position = state.velocity;

  // Sprint 2: Compute all forces
  Eigen::Vector3d F_gravity(0.0, 0.0, -state.totalMass * GRAVITY);
  Eigen::Vector3d F_drag = Eigen::Vector3d::Zero();
  Eigen::Vector3d F_thrust = Eigen::Vector3d::Zero();
  Eigen::Vector3d F_finLift = Eigen::Vector3d::Zero();

  // Try casting to SmartArtillery first (it extends DumbArtillery)
  auto *smartArtillery = dynamic_cast<SmartArtillery *>(proj);
  auto *dumbArtillery = dynamic_cast<DumbArtillery *>(proj);

  if (smartArtillery) {
    // Get basic forces (drag and thrust)
    F_drag = smartArtillery->getAero().computeDragForce(state.velocity,
                                                        state.position.z());
    F_thrust = smartArtillery->getPropulsion().computeThrustForce(
        state.elapsedTime, state.velocity);

    // Add fin forces if guidance is enabled
    if (smartArtillery->isGuidanceEnabled()) {
      // SmartArtillery computes fin lift using its guidance/navigation/control
      // systems
      F_finLift = smartArtillery->computeFinLift(state);
    }
  } else if (dumbArtillery) {
    // Original dumb artillery logic
    F_drag = dumbArtillery->getAero().computeDragForce(state.velocity,
                                                       state.position.z());
    F_thrust = dumbArtillery->getPropulsion().computeThrustForce(
        state.elapsedTime, state.velocity);
  }

  // Total force
  Eigen::Vector3d F_total = F_gravity + F_drag + F_thrust + F_finLift;

  // d(vel)/dt = acceleration = F_total / m_current
  if (state.totalMass > 1e-6) {
    output.velocity = F_total / state.totalMass;

    // Update G-load for SmartArtillery telemetry
    if (smartArtillery) {
      smartArtillery->setCurrentGLoad(output.velocity);
    }
  } else {
    output.velocity = Eigen::Vector3d::Zero();
  }

  // d(mass)/dt and d(fuel)/dt from fuel consumption
  double fuelBurnRate = 0.0;
  if (dumbArtillery) {
    fuelBurnRate = dumbArtillery->getPropulsion().isBurning(state.elapsedTime)
                       ? dumbArtillery->getPropulsion().getFuelConsumed(1.0)
                       : 0.0; // mass flow rate
    // Actually get the mass flow rate from config
    if (dumbArtillery->getPropulsion().isBurning(state.elapsedTime)) {
      // We need mass flow rate - let's compute it from consumed fuel
      double dt_small = 0.001;
      double fuel1 =
          dumbArtillery->getPropulsion().getFuelConsumed(state.elapsedTime);
      double fuel2 = dumbArtillery->getPropulsion().getFuelConsumed(
          state.elapsedTime + dt_small);
      fuelBurnRate = (fuel2 - fuel1) / dt_small;
    }
  }

  output.totalMass = -fuelBurnRate;
  output.fuelMass = -fuelBurnRate;

  // d(time)/dt = 1
  output.elapsedTime = 1.0;

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

    // RK4 Integration with projectile reference
    // k1
    StateVector k1 = derivative(current, artillery);

    // k2
    StateVector k2State = current + k1 * (0.5 * dt);
    StateVector k2 = derivative(k2State, artillery);

    // k3
    StateVector k3State = current + k2 * (0.5 * dt);
    StateVector k3 = derivative(k3State, artillery);

    // k4
    StateVector k4State = current + k3 * dt;
    StateVector k4 = derivative(k4State, artillery);

    // Final state
    StateVector nextState =
        current + (k1 + k2 * 2.0 + k3 * 2.0 + k4) * (dt / 6.0);

    // Ensure fuel doesn't go negative
    if (nextState.fuelMass < 0.0) {
      nextState.fuelMass = 0.0;
    }
    nextState.totalMass =
        artillery->getMass().getDryMass() + nextState.fuelMass;

    artillery->setState(nextState);
  }
}
