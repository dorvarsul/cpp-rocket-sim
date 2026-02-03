#include "../src/core/DumbArtillery.h"
#include "../src/core/SimulationWorld.h"
#include <cmath>
#include <iomanip>
#include <iostream>

void test_terminal_velocity() {
  std::cout << "=====================================" << std::endl;
  std::cout << "Test 1: Terminal Velocity Check" << std::endl;
  std::cout << "=====================================" << std::endl;
  std::cout << "Drop a projectile from 10,000m with minimal "
            << "initial velocity." << std::endl;
  std::cout << "Expected: Velocity should plateau at terminal velocity."
            << std::endl;
  std::cout << std::endl;

  SimulationWorld world;
  LaunchConfig config;
  config.elevation_deg = -89.0; // Nearly straight down
  config.azimuth_deg = 0.0;
  config.dryMass_kg = 50.0;
  config.fuelMass_kg = 0.0; // No fuel
  config.referenceArea_m2 = 0.01;
  config.thrust_N = 0.0; // No thrust
  config.burnDuration_s = 0.0;
  config.massFlowRate_kgps = 0.0;

  auto projectile = std::make_unique<DumbArtillery>(config);

  // Set initial altitude to 10,000m with small downward velocity
  StateVector initialState = projectile->getState();
  initialState.position.z() = 10001.0;
  initialState.velocity =
      Eigen::Vector3d(0.0, 0.0, -10.0); // Downward velocity start
  projectile->setState(initialState);

  world.addProjectile(std::move(projectile));

  double dt = 0.01;
  int steps = 0;
  double prevSpeed = 0.0;
  double terminalVelocity = 0.0;
  bool foundTerminal = false;

  // Run until landed or max time
  while (steps < 150000) { // 1500 seconds max
    world.step(dt);
    const auto &projectiles = world.getProjectiles();
    StateVector current = projectiles[0]->getState();

    double speed = current.velocity.norm();

    // Check if velocity is plateauing
    if (steps > 500 && std::abs(speed - prevSpeed) < 0.01) {
      if (!foundTerminal) {
        terminalVelocity = speed;
        foundTerminal = true;
        std::cout << "Terminal velocity reached at t=" << steps * dt
                  << "s: " << terminalVelocity << " m/s" << std::endl;
      }
    }

    prevSpeed = speed;

    if (projectiles[0]->hasLanded())
      break;
    steps++;
  }

  // Calculate theoretical terminal velocity: v_term = sqrt(2*m*g / (rho * Cd *
  // A)) At low altitude: rho ≈ 1.225 kg/m³, Cd ≈ 0.15, A = 0.01 m², m = 50 kg
  double rho = 1.225;
  double Cd = 0.15;
  double A = 0.01;
  double m = 50.0;
  double g = 9.81;
  double theoretical = std::sqrt((2.0 * m * g) / (rho * Cd * A));

  std::cout << std::endl;
  std::cout << "Results:" << std::endl;
  std::cout << "  Measured terminal velocity: " << terminalVelocity << " m/s"
            << std::endl;
  std::cout << "  Theoretical terminal velocity: " << theoretical << " m/s"
            << std::endl;
  std::cout << "  Error: " << std::abs(terminalVelocity - theoretical) << " m/s"
            << std::endl;
  std::cout << "  Percent error: "
            << (std::abs(terminalVelocity - theoretical) / theoretical * 100.0)
            << "%" << std::endl;

  bool passed =
      foundTerminal && (std::abs(terminalVelocity - theoretical) / theoretical <
                        0.15); // 15% tolerance
  std::cout << std::endl;
  if (passed) {
    std::cout << "  [PASS]" << std::endl;
  } else {
    std::cout << "  [FAIL]" << std::endl;
  }
  std::cout << "=====================================" << std::endl
            << std::endl;
}

void test_max_q() {
  std::cout << "=====================================" << std::endl;
  std::cout << "Test 2: Max Q Verification" << std::endl;
  std::cout << "=====================================" << std::endl;
  std::cout << "Launch rocket straight up with high thrust." << std::endl;
  std::cout << "Expected: Drag force should peak at Max Q altitude."
            << std::endl;
  std::cout << std::endl;

  SimulationWorld world;
  LaunchConfig config;
  config.elevation_deg = 90.0; // Straight up
  config.azimuth_deg = 0.0;
  config.dryMass_kg = 100.0;
  config.fuelMass_kg = 50.0;
  config.referenceArea_m2 = 0.02;
  config.thrust_N = 5000.0; // High thrust
  config.burnDuration_s = 15.0;
  config.massFlowRate_kgps = 50.0 / 15.0;

  world.addProjectile(std::make_unique<DumbArtillery>(config));

  double dt = 0.01;
  int steps = 0;
  double maxDragForce = 0.0;
  double maxDragAltitude = 0.0;
  double maxDragTime = 0.0;

  // Run simulation
  while (steps < 30000) { // 300 seconds max
    world.step(dt);
    const auto &projectiles = world.getProjectiles();

    if (projectiles[0]->hasLanded())
      break;

    auto *artillery = dynamic_cast<DumbArtillery *>(projectiles[0].get());
    if (artillery) {
      double dragForce = artillery->getDragForce().norm();
      StateVector state = artillery->getState();

      if (dragForce > maxDragForce) {
        maxDragForce = dragForce;
        maxDragAltitude = state.position.z();
        maxDragTime = steps * dt;
      }
    }

    steps++;
  }

  std::cout << "Results:" << std::endl;
  std::cout << "  Max drag force (Max Q): " << maxDragForce << " N"
            << std::endl;
  std::cout << "  Occurred at altitude: " << maxDragAltitude << " m"
            << std::endl;
  std::cout << "  Occurred at time: " << maxDragTime << " s" << std::endl;

  // Pass criteria: Max Q should occur and be substantial
  bool passed = (maxDragForce > 100.0) && (maxDragAltitude > 100.0);
  std::cout << std::endl;
  if (passed) {
    std::cout << "  [PASS] Max Q detected at altitude " << maxDragAltitude
              << " m" << std::endl;
  } else {
    std::cout << "  [FAIL] Max Q not properly detected" << std::endl;
  }
  std::cout << "=====================================" << std::endl
            << std::endl;
}

void test_mass_to_range() {
  std::cout << "=====================================" << std::endl;
  std::cout << "Test 3: Mass-to-Range Correlation" << std::endl;
  std::cout << "=====================================" << std::endl;
  std::cout << "Run two simulations with different fuel masses." << std::endl;
  std::cout << "Expected: More fuel should result in different range."
            << std::endl;
  std::cout << std::endl;

  // Simulation A: 10kg fuel
  std::cout << "Running Simulation A (10kg fuel)..." << std::endl;
  SimulationWorld worldA;
  LaunchConfig configA;
  configA.elevation_deg = 45.0;
  configA.azimuth_deg = 0.0;
  configA.dryMass_kg = 50.0;
  configA.fuelMass_kg = 10.0;
  configA.referenceArea_m2 = 0.01;
  configA.thrust_N = 2000.0;
  configA.burnDuration_s = 5.0;
  configA.massFlowRate_kgps = 10.0 / 5.0;

  worldA.addProjectile(std::make_unique<DumbArtillery>(configA));

  double dt = 0.01;
  int stepsA = 0;
  while (stepsA < 50000) {
    worldA.step(dt);
    if (worldA.getProjectiles()[0]->hasLanded())
      break;
    stepsA++;
  }

  StateVector finalA = worldA.getProjectiles()[0]->getState();
  double rangeA = std::sqrt(finalA.position.x() * finalA.position.x() +
                            finalA.position.y() * finalA.position.y());

  std::cout << "  Sim A final range: " << rangeA << " m" << std::endl;
  std::cout << "  Sim A final fuel: " << finalA.fuelMass << " kg" << std::endl;

  // Simulation B: 100kg fuel
  std::cout << std::endl << "Running Simulation B (100kg fuel)..." << std::endl;
  SimulationWorld worldB;
  LaunchConfig configB;
  configB.elevation_deg = 45.0;
  configB.azimuth_deg = 0.0;
  configB.dryMass_kg = 50.0;
  configB.fuelMass_kg = 100.0;
  configB.referenceArea_m2 = 0.01;
  configB.thrust_N = 2000.0;
  configB.burnDuration_s = 50.0;
  configB.massFlowRate_kgps = 100.0 / 50.0;
  configB.thrust_N = 3000.0; // Increased to ensure liftoff (T > W)
  configB.burnDuration_s = 50.0;
  configB.massFlowRate_kgps = 100.0 / 50.0;

  worldB.addProjectile(std::make_unique<DumbArtillery>(configB));

  int stepsB = 0;
  while (stepsB < 100000) {
    worldB.step(dt);
    if (worldB.getProjectiles()[0]->hasLanded())
      break;
    stepsB++;
  }

  StateVector finalB = worldB.getProjectiles()[0]->getState();
  double rangeB = std::sqrt(finalB.position.x() * finalB.position.x() +
                            finalB.position.y() * finalB.position.y());

  std::cout << "  Sim B final range: " << rangeB << " m" << std::endl;
  std::cout << "  Sim B final fuel: " << finalB.fuelMass << " kg" << std::endl;

  std::cout << std::endl << "Comparison:" << std::endl;
  std::cout << "  Range difference: " << std::abs(rangeB - rangeA) << " m"
            << std::endl;
  std::cout << "  Percent change: " << ((rangeB - rangeA) / rangeA * 100.0)
            << "%" << std::endl;

  // Pass criteria: Fuel depleted and ranges are different
  bool fuelDepletedA = finalA.fuelMass < 0.1;
  bool fuelDepletedB = finalB.fuelMass < 0.1;
  bool rangesDifferent =
      std::abs(rangeB - rangeA) > 100.0; // Substantial difference

  bool passed = fuelDepletedA && fuelDepletedB && rangesDifferent;
  std::cout << std::endl;
  if (passed) {
    std::cout << "  [PASS] Mass depletion verified, range correlation confirmed"
              << std::endl;
  } else {
    std::cout << "  [FAIL] ";
    if (!fuelDepletedA || !fuelDepletedB)
      std::cout << "Fuel not fully depleted. ";
    if (!rangesDifferent)
      std::cout << "Ranges not sufficiently different. ";
    std::cout << std::endl;
  }
  std::cout << "=====================================" << std::endl
            << std::endl;
}

int main() {
  std::cout << std::endl;
  std::cout << "========================================" << std::endl;
  std::cout << "   SPRINT 2 TEST SUITE" << std::endl;
  std::cout << "   Atmospheric & Mass Dynamics" << std::endl;
  std::cout << "========================================" << std::endl;
  std::cout << std::endl;

  test_terminal_velocity();
  test_max_q();
  test_mass_to_range();

  std::cout << "========================================" << std::endl;
  std::cout << "   All Sprint 2 tests completed!" << std::endl;
  std::cout << "========================================" << std::endl;
  std::cout << std::endl;

  return 0;
}
