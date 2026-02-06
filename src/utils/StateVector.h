#pragma once
#include <Eigen/Dense>

struct StateVector {
  Eigen::Vector3d position; // [x, y, z] in ENU (East-North-Up) meters
  Eigen::Vector3d velocity; // [vx, vy, vz] in m/s
  double totalMass;         // Total mass (kg) - dry + fuel
  double fuelMass;          // Remaining fuel mass (kg)
  double elapsedTime;       // Simulation time since launch (s)

  static StateVector Zero() {
    StateVector sv;
    sv.position = Eigen::Vector3d::Zero();
    sv.velocity = Eigen::Vector3d::Zero();
    sv.totalMass = 0.0;
    sv.fuelMass = 0.0;
    sv.elapsedTime = 0.0;
    return sv;
  }

  StateVector operator+(const StateVector &other) const {
    StateVector result;
    result.position = position + other.position;
    result.velocity = velocity + other.velocity;
    result.totalMass = totalMass + other.totalMass;
    result.fuelMass = fuelMass + other.fuelMass;
    result.elapsedTime = elapsedTime + other.elapsedTime;
    return result;
  }

  StateVector operator*(double scalar) const {
    StateVector result;
    result.position = position * scalar;
    result.velocity = velocity * scalar;
    result.totalMass = totalMass * scalar;
    result.fuelMass = fuelMass * scalar;
    result.elapsedTime = elapsedTime * scalar;
    return result;
  }
};

inline StateVector operator*(double scalar, const StateVector &state) {
  return state * scalar;
}
