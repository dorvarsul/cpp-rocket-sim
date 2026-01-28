#include "DumbArtillery.h"
#include <cmath>
#include <numbers>

DumbArtillery::DumbArtillery(const LaunchConfig &config)
    : m_atmosphere(), m_aero(config.referenceArea_m2, m_atmosphere),
      m_propulsion(ThrustProfile{
          config.thrust_N, config.burnDuration_s, config.massFlowRate_kgps,
          Eigen::Vector3d::Zero()}), // Temp, will set below
      m_mass(config.dryMass_kg, config.fuelMass_kg) {

  // Calculate launch direction from elevation and azimuth
  double el_rad = config.elevation_deg * std::numbers::pi / 180.0;
  double az_rad = config.azimuth_deg * std::numbers::pi / 180.0;

  // ENU coordinates: X=East, Y=North, Z=Up
  double horiz = std::cos(el_rad);
  Eigen::Vector3d launchDir(horiz * std::cos(az_rad), // X (East)
                            horiz * std::sin(az_rad), // Y (North)
                            std::sin(el_rad)          // Z (Up)
  );

  // Update propulsion with correct launch direction
  m_propulsion =
      PropulsionSystem(ThrustProfile{config.thrust_N, config.burnDuration_s,
                                     config.massFlowRate_kgps, launchDir});

  // Initial position is origin
  m_state.position = Eigen::Vector3d::Zero();

  // Rockets start from rest - velocity is generated purely by thrust
  // This is realistic rocket physics
  m_state.velocity = Eigen::Vector3d::Zero();

  // Sprint 2: Initialize mass and time
  m_state.totalMass = m_mass.getTotalMass();
  m_state.fuelMass = m_mass.getRemainingFuel();
  m_state.elapsedTime = 0.0;

  // Record initial state
  m_history.push_back(m_state);
}

void DumbArtillery::update(double) {
  // This function is purely to satisfy interface if needed by other systems,
  // but actual integration happens in SimulationWorld via RK4 which sets state
  // directly. However, if we were doing internal integration, it would go here.
  // For this architecture, SimulationWorld is the "Solver" driving the
  // projectiles.
}

StateVector DumbArtillery::getState() const { return m_state; }

std::vector<StateVector> DumbArtillery::getHistory() const { return m_history; }

bool DumbArtillery::hasLanded() const { return m_landed; }

void DumbArtillery::setState(const StateVector &newState) {
  m_state = newState;

  // Update fuel mass in MassProperties
  m_mass.setRemainingFuel(m_state.fuelMass);

  // Check ground collision
  if (m_state.position.z() <= 0.0) {
    m_state.position.z() = 0.0;
    m_state.velocity = Eigen::Vector3d::Zero();
    m_landed = true;
  }

  m_history.push_back(m_state);
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
