#include "../src/simulation/SimulationWorld.h"
#include "../src/simulation/SmartArtillery.h"
#include "../src/utils/LaunchAngleSolver.h"
#include <cmath>
#include <iostream>
#include <vector>

void run_test(double targetX) {
  std::cout << "\n=== Testing Target X = " << targetX << " m ===" << std::endl;

  // 1. Setup Configuration
  LaunchConfig config;
  config.dryMass_kg = 45.5;
  config.fuelMass_kg = 20.5;
  config.referenceArea_m2 = 0.025;
  config.thrust_N = 24000.0;
  config.burnDuration_s = 2.0;
  config.massFlowRate_kgps = 10.5;
  config.elevation_deg = 45.0; // Will be optimized
  config.azimuth_deg = 0.0;

  GuidanceConfig guidanceConfig;
  guidanceConfig.proNavGain = 3.0;
  guidanceConfig.convergenceRadius_m = 10.0;
  guidanceConfig.targetPosition = Eigen::Vector3d(targetX, 0, 0);

  ControlLimits controlLimits;
  controlLimits.maxFinDeflection_deg = 20.0;
  controlLimits.maxGLoad = 10.0;

  // 2. Solve for Launch Angle
  LaunchAngleSolver solver;
  Eigen::Vector3d targetPos(targetX, 0, 0);
  auto [elev, az] = solver.calculateLaunchAngles(config, targetPos);

  // Debug: Check what the solver PREDICTS for this angle
  Eigen::Vector3d predictedLanding =
      solver.estimateLandingPosition(config, elev, az);
  std::cout << "Solver Calculated Elevation: " << elev << " deg" << std::endl;
  std::cout << "Solver Predicted Landing X: " << predictedLanding.x() << " m"
            << std::endl;

  // config.elevation_deg = elev * 0.75; // The hack currently in main.cpp
  config.elevation_deg = elev; // Use pure solver for baseline physics check
  config.azimuth_deg = az;

  std::cout << "Launch Elevation: " << config.elevation_deg << " deg"
            << std::endl;

  // 3. Setup Simulation
  SimulationWorld world;
  auto projectile = std::make_unique<SmartArtillery>(config, guidanceConfig,
                                                     controlLimits, false);
  // Guidance enabled by default (Predictive Guidance)
  world.addProjectile(std::move(projectile));

  // 4. Run Simulation
  double t = 0.0;
  const double dt = 0.01;
  const double max_t = 300.0;

  bool landed = false;
  Eigen::Vector3d finalPos = Eigen::Vector3d::Zero();

  while (t < max_t) {
    world.step(dt);
    t += dt;

    // Check landing
    const auto &projs = world.getProjectiles();
    if (!projs.empty()) {
      if (projs[0]->hasLanded()) {
        finalPos = projs[0]->getState().position;
        landed = true;
        break;
      }
    }
  }

  // 5. Report Results
  if (landed) {
    double error = (finalPos - targetPos).norm();
    std::cout << "Landed at X: " << finalPos.x() << " m" << std::endl;
    std::cout << "Target X: " << targetX << " m" << std::endl;
    std::cout << "Error: " << error << " m" << std::endl;
  } else {
    std::cout << "Rocket did not land within " << max_t << "s" << std::endl;
  }
}

int main() {
  run_test(5000.0);
  run_test(10000.0);
  return 0;
}
