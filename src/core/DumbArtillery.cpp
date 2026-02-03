#include "DumbArtillery.h"
#include <cmath>
#include <numbers>

DumbArtillery::DumbArtillery(const LaunchConfig &config)
    : m_state(StateVector::Zero()), m_atmosphere(),
      m_aero(config.referenceArea_m2, m_atmosphere),
      m_propulsion(ThrustProfile{
          config.thrust_N, config.burnDuration_s, config.massFlowRate_kgps,
          Eigen::Vector3d::UnitZ()}), // Tmp, overwritten below
      m_mass(config.dryMass_kg, config.fuelMass_kg) {

  // Calculate launch direction from elevation and azimuth
  double el_rad = config.elevation_deg * std::numbers::pi / 180.0;
  double az_rad = config.azimuth_deg * std::numbers::pi / 180.0;

  // ENU coordinates: X=East, Y=North, Z=Up
  double xy_proj = std::cos(el_rad);
  m_launchDirection = Eigen::Vector3d(xy_proj * std::sin(az_rad), // X (East)
                                      xy_proj * std::cos(az_rad), // Y (North)
                                      std::sin(el_rad)            // Z (Up)
                                      )
                          .normalized();

  // Update propulsion profile with correct direction
  m_propulsion = PropulsionSystem(
      ThrustProfile{config.thrust_N, config.burnDuration_s,
                    config.massFlowRate_kgps, m_launchDirection});

  m_state.totalMass = config.dryMass_kg + config.fuelMass_kg;
  m_state.fuelMass = config.fuelMass_kg;
  m_state.position = Eigen::Vector3d(0, 0, 0); // Start at origin
  m_state.velocity = Eigen::Vector3d(0, 0, 0);

  // Record initial state
  m_history.push_back(m_state);
}

void DumbArtillery::update(double) {
  // This function is purely to satisfy interface if needed by other systems,
  // but actual integration happens in SimulationWorld via RK4 which sets state
  // directly. However, if we were doing internal integration, it would go here.
}

StateVector DumbArtillery::getState() const { return m_state; }

const std::vector<StateVector> &DumbArtillery::getHistory() const {
  return m_history;
}

bool DumbArtillery::hasLanded() const { return m_landed; }

void DumbArtillery::setState(const StateVector &newState) {
  m_state = newState;

  // Update fuel mass in MassProperties
  m_mass.setRemainingFuel(m_state.fuelMass);

  // Check ground collision
  // Check ground collision
  if (m_state.position.z() <= 0.0) {
    // Only register landing if:
    // 1. We have been flying for a bit (avoid false triggering at launch t=0)
    // 2. We are actually moving downwards (velocity.z < 0)
    // 3. Or if we are fully stopped at t=0 (which is initial state, handled by
    // !landed init)

    // If we are launching (t < 1.0) or moving up (v > 0), don't land!
    if (m_state.velocity.z() < -0.1 ||
        (m_state.elapsedTime > 0.5 && m_state.velocity.z() < 0.1)) {
      m_state.position.z() = 0.0;
      m_state.velocity = Eigen::Vector3d::Zero();
      m_landed = true;
    } else {
      // We are on launchpad or skimming ground
      // Clamp to 0 but allow motion
      if (m_state.position.z() < 0.0)
        m_state.position.z() = 0.0;
      // Don't kill velocity if positive!
      if (m_state.velocity.z() < 0.0)
        m_state.velocity.z() = 0.0; // Prevent falling through
      m_landed = false;
    }
  }

  m_history.push_back(m_state);
}

// Compute total forces: Gravity + Drag + Thrust
Eigen::Vector3d
DumbArtillery::computeForces(const StateVector &state,
                             const Eigen::Vector3d &windVelocity) const {
  const double GRAVITY = 9.81;
  Eigen::Vector3d F_gravity(0.0, 0.0, -state.totalMass * GRAVITY);

  // Wind correction: Aero forces depend on Airspeed
  Eigen::Vector3d relativeVelocity = state.velocity - windVelocity;

  // Drag uses relative velocity
  Eigen::Vector3d F_drag =
      m_aero.computeDragForce(relativeVelocity, state.position.z());

  // Thrust is independent of wind
  // Calculate Launch Rail Direction (Initial Orientation)
  // We can infer this from LaunchConfig, but we don't store LaunchConfig
  // indefinitely? Actually we only need to construct it once or store it. For
  // now, let's reconstruct it from the initial state if needed, OR since we
  // don't have the config here readily available (it's in m_propulsion
  // profile?), we can use the state.velocity at t=0? No, velocity starts at 0.
  // We need to calculate it from Elevation/Azimuth.
  // Wait, DumbArtillery doesn't store launch config.
  // Let's store the initial launch vector in the class.

  Eigen::Vector3d launchDir = Eigen::Vector3d::Zero();
  // We need to access m_launchVector. Adding it to header first.
  launchDir = m_launchDirection; // Will add to header

  Eigen::Vector3d F_thrust = m_propulsion.computeThrustForce(
      state.elapsedTime, state.velocity, launchDir);

  return F_gravity + F_drag + F_thrust;
}

double DumbArtillery::getMassFlowRate(double time) const {
  if (m_propulsion.isBurning(time)) {
    // We use numerical differentiation in SimulationWorld previously,
    // but here we can just ask the propulsion system or get it from config if
    // accessible. Actually, PropulsionSystem stores massFlowRate in
    // ThrustProfile, but it seems it's private inside PropulsionSystem? Let's
    // check PropulsionSystem.h. If not accessible, we can replicate logic or
    // modify PropulsionSystem.

    // However, dumbArtillery matches config.massFlowRate_kgps during burn.
    // Let's deduce it from getFuelConsumed difference for now to match behavior
    // exactly without peeking internals if not exposed. Or better, since
    // default PropulsionSystem might not expose rate.

    // Let's assume constant rate for now based on burn duration, or use the
    // small delta trick if we want to be generic over profile. The original
    // code used:
    /*
      double dt_small = 0.001;
      double fuel1 =
      dumbArtillery->getPropulsion().getFuelConsumed(state.elapsedTime); double
      fuel2 = dumbArtillery->getPropulsion().getFuelConsumed(state.elapsedTime +
      dt_small); fuelBurnRate = (fuel2 - fuel1) / dt_small;
    */

    double dt_small = 0.001;
    double fuel1 = m_propulsion.getFuelConsumed(time);
    double fuel2 = m_propulsion.getFuelConsumed(time + dt_small);
    return (fuel2 - fuel1) / dt_small;
  }
  return 0.0;
}

double DumbArtillery::getMachNumber() const {
  return m_aero.getMachNumber(m_state.velocity, m_state.position.z());
}

Eigen::Vector3d DumbArtillery::getDragForce() const {
  return m_aero.computeDragForce(m_state.velocity, m_state.position.z());
}

Eigen::Vector3d DumbArtillery::getThrustForce() const {
  return m_propulsion.computeThrustForce(m_state.elapsedTime, m_state.velocity);
}

double DumbArtillery::getCurrentMass() const { return m_state.totalMass; }

double DumbArtillery::getRemainingFuel() const { return m_state.fuelMass; }
