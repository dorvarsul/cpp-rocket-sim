#pragma once
#include <Eigen/Dense>

struct StateVector {
  Eigen::Vector3d position; // [x, y, z] in ENU (East-North-Up) meters
  Eigen::Vector3d velocity; // [vx, vy, vz] in m/s

  static StateVector Zero() {
    return {Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()};
  }

  StateVector operator+(const StateVector &other) const {
    return {position + other.position, velocity + other.velocity};
  }

  StateVector operator*(double scalar) const {
    return {position * scalar, velocity * scalar};
  }
};

inline StateVector operator*(double scalar, const StateVector &state) {
  return state * scalar;
}
