#pragma once
#include <Eigen/Dense>

struct GuidanceConfig {
  double proNavGain = 3.0;
  double convergenceRadius_m = 10.0;
  Eigen::Vector3d targetPosition = Eigen::Vector3d::Zero();
};
