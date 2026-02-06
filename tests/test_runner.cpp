#include "../src/simulation/DumbArtillery.h"
#include "../src/simulation/SimulationWorld.h"
#include <cmath>
#include <iomanip>
#include <iostream>

void test_vacuum_45() {
  std::cout << "Test 1: 45-Degree Vacuum Check" << std::endl;
  SimulationWorld world;
  LaunchConfig config;
  config.elevation_deg = 45.0;
  config.azimuth_deg = 0.0;

  world.addProjectile(std::make_unique<DumbArtillery>(config));

  // Step until landed
  double dt = 0.01;
  int steps = 0;
  // Safety limit 20 seconds (flight time should be ~14.4s)
  while (steps < 20000) {
    world.step(dt);
    const auto &projectiles = world.getProjectiles();
    if (projectiles[0]->hasLanded())
      break;
    steps++;
  }

  auto history = world.getProjectiles()[0]->getHistory();
  StateVector finalState = history.back();

  double range = finalState.position.x(); // Azimuth 0 => X is range
  double expected = (100.0 * 100.0) / 9.81;

  std::cout << "  Final Range: " << range << " m" << std::endl;
  std::cout << "  Expected:    " << expected << " m" << std::endl;
  std::cout << "  Error:       " << std::abs(range - expected) << " m"
            << std::endl;

  if (std::abs(range - expected) <
      2.0) { // Allow small error due to discrete steps
    std::cout << "  [PASS]" << std::endl;
  } else {
    std::cout << "  [FAIL]" << std::endl;
  }
  std::cout << "----------------------------------------" << std::endl;
}

void test_vertical_90() {
  std::cout << "Test 2: Vertical Return (State Vector Test)" << std::endl;
  SimulationWorld world;
  LaunchConfig config;
  config.elevation_deg = 90.0;
  config.azimuth_deg = 0.0;

  world.addProjectile(std::make_unique<DumbArtillery>(config));

  double dt = 0.01;
  int steps = 0;
  bool ghostForceDetected = false;

  while (steps < 25000) { // ~20.4s flight time
    world.step(dt);
    const auto &projectiles = world.getProjectiles();
    StateVector current = projectiles[0]->getState();

    if (std::abs(current.position.x()) > 1e-4 ||
        std::abs(current.position.y()) > 1e-4) {
      ghostForceDetected = true;
      std::cout << "  Drift detected at t=" << steps * dt << ": "
                << current.position.x() << ", " << current.position.y()
                << std::endl;
    }

    if (projectiles[0]->hasLanded())
      break;
    steps++;
  }

  StateVector finalState = world.getProjectiles()[0]->getState();
  std::cout << "  Final Position: " << finalState.position.transpose()
            << std::endl;

  if (!ghostForceDetected && std::abs(finalState.position.x()) < 1e-4 &&
      std::abs(finalState.position.y()) < 1e-4) {
    std::cout << "  [PASS]" << std::endl;
  } else {
    std::cout << "  [FAIL]" << std::endl;
  }
  std::cout << "----------------------------------------" << std::endl;
}

void test_oop_extensibility() {
  std::cout << "Test 3: OOP Extensibility" << std::endl;
  SimulationWorld world;

  LaunchConfig c1;
  c1.elevation_deg = 45.0;
  c1.azimuth_deg = 0.0;

  LaunchConfig c2;
  c2.elevation_deg = 60.0;
  c2.azimuth_deg = 90.0; // North

  world.addProjectile(std::make_unique<DumbArtillery>(c1));
  world.addProjectile(std::make_unique<DumbArtillery>(c2));

  double dt = 0.01;
  for (int i = 0; i < 100; ++i)
    world.step(dt);

  const auto &projectiles = world.getProjectiles();
  StateVector s1 = projectiles[0]->getState();
  StateVector s2 = projectiles[1]->getState();

  std::cout << "  P1 (East) X: " << s1.position.x()
            << ", Y: " << s1.position.y() << std::endl;
  std::cout << "  P2 (North) X: " << s2.position.x()
            << ", Y: " << s2.position.y() << std::endl;

  // P1 should be primarily X, P2 primarily Y
  bool p1_correct = s1.position.x() > 10.0 && std::abs(s1.position.y()) < 1e-4;
  bool p2_correct = std::abs(s2.position.x()) < 1e-4 && s2.position.y() > 10.0;

  if (p1_correct && p2_correct) {
    std::cout << "  [PASS] Independent trajectories confirmed." << std::endl;
  } else {
    std::cout << "  [FAIL] Interference or wrong setup." << std::endl;
  }
  std::cout << "----------------------------------------" << std::endl;
}

int main() {
  test_vacuum_45();
  test_vertical_90();
  test_oop_extensibility();
  return 0;
}
