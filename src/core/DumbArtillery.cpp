#include "DumbArtillery.h"
#include <cmath>
#include <numbers>

DumbArtillery::DumbArtillery(const LaunchConfig &config) {
  // Convert degrees to radians
  double el_rad = config.elevation_deg * std::numbers::pi / 180.0;
  double az_rad = config.azimuth_deg * std::numbers::pi /
                  180.0; // 0 = East, 90 = North (CCW)

  // Initial position is origin
  m_state.position = Eigen::Vector3d::Zero();

  // Calculate initial velocity components using ENU coordinates
  // Z is Up (sin elevation)
  // Horizontal component is cos elevation
  // X is East (cos azimuth)
  // Y is North (sin azimuth)
  double v_z = config.muzzle_velocity_mps * std::sin(el_rad);
  double v_horiz = config.muzzle_velocity_mps * std::cos(el_rad);

  double v_x = v_horiz * std::cos(az_rad);
  double v_y = v_horiz * std::sin(az_rad);

  m_state.velocity = Eigen::Vector3d(v_x, v_y, v_z);

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

  // Check ground collision
  if (m_state.position.z() <= 0.0) {
    m_state.position.z() = 0.0;
    m_state.velocity = Eigen::Vector3d::Zero();
    m_landed = true;
  }

  m_history.push_back(m_state);
}
