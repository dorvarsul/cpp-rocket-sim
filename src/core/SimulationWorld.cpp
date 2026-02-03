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

StateVector SimulationWorld::derivative(const StateVector &state,
                                        IProjectile *proj) const {
  StateVector output;

  // d(pos)/dt = velocity
  output.position = state.velocity;

  // Force Calculation via Polymorphism
  Eigen::Vector3d F_total = proj->computeForces(state, m_windVelocity);

  // d(vel)/dt = acceleration = F_total / m_current
  if (state.totalMass > 1e-6) {
    output.velocity = F_total / state.totalMass;
  } else {
    output.velocity = Eigen::Vector3d::Zero();
  }

  // d(mass)/dt and d(fuel)/dt from projectile specifics
  // Use new interface method instead of casting
  double fuelBurnRate = proj->getMassFlowRate(state.elapsedTime);

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

    StateVector current = proj->getState();

    // RK4 Integration with projectile reference
    // k1
    StateVector k1 = derivative(current, proj.get());

    // k2
    StateVector k2State = current + k1 * (0.5 * dt);
    StateVector k2 = derivative(k2State, proj.get());

    // k3
    StateVector k3State = current + k2 * (0.5 * dt);
    StateVector k3 = derivative(k3State, proj.get());

    // k4
    StateVector k4State = current + k3 * dt;
    StateVector k4 = derivative(k4State, proj.get());

    // Final state
    StateVector nextState =
        current + (k1 + k2 * 2.0 + k3 * 2.0 + k4) * (dt / 6.0);

    // Ensure fuel doesn't go negative
    if (nextState.fuelMass < 0.0) {
      nextState.fuelMass = 0.0;
    }
    // Update total mass based on fuel (assuming dry mass is constant)
    // We need to know dry mass or just let the projectile handle mass
    // consistency in setState? Projectile's setState in DumbArtillery handles
    // totalMass = dry + fuel. However, here we calculated d(totalMass)/dt =
    // -burnRate. So nextState.totalMass should be correct relative to
    // integration. Refinement: DumbArtillery::setState re-calculates totalMass
    // from fuelMass + dryMass (stored internally). So technically,
    // nextState.totalMass computed here via integration might drift from
    // (dry+fuel) if not careful? Actually, DumbArtillery::setState OVERWRITES
    // totalList using its internal dry mass + new fuel mass. So we just need to
    // ensure fuelMass is correct.

    // Pass to projectile
    proj->setState(nextState);

    // Telemetry Update
    // Calculate effective acceleration for this step for telemetry display
    // Acceleration ~ (v_new - v_old) / dt
    if (dt > 1e-9) {
      Eigen::Vector3d accel = (nextState.velocity - current.velocity) / dt;
      proj->updateTelemetry(accel);
    }
  }
}
