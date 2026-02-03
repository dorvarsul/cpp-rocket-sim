#include "../src/core/LaunchAngleSolver.h"
#include <iostream>

int main() {
  LaunchConfig config;
  config.elevation_deg = 45.0;
  config.azimuth_deg = 0.0;
  config.dryMass_kg = 100.0;
  config.fuelMass_kg = 50.0;
  config.referenceArea_m2 = 0.02;
  config.thrust_N = 5000.0;
  config.burnDuration_s = 15.0;
  config.massFlowRate_kgps = 50.0 / 15.0;

  Eigen::Vector3d target(1000.0, 1000.0, 0.0);

  std::cout << "Testing LaunchAngleSolver with default config..." << std::endl;
  std::cout << "Target: " << target.transpose() << std::endl;

  LaunchAngleSolver solver;
  bool reachable = solver.isTargetReachable(config, target);

  std::cout << "Reachable: " << (reachable ? "YES" : "NO") << std::endl;
  std::cout << "Calculated Range: " << solver.getCalculatedRange() << std::endl;

  return 0;
}
