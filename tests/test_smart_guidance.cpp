#include "../src/simulation/SimulationWorld.h"
#include "../src/simulation/SmartArtillery.h"
#include <iomanip>
#include <iostream>
#include <memory>

void test_drop_physics() {
  std::cout << "--- Drop Test (Physics Debug) ---" << std::endl;
  SimulationWorld world;
  LaunchConfig config;
  config.elevation_deg = 90;
  config.dryMass_kg = 50;
  config.fuelMass_kg = 0;
  config.thrust_N = 0;
  config.referenceArea_m2 = 0.01;

  // Config irrelevant for guidance/control in drop test
  GuidanceConfig gConfig;
  ControlLimits cLimits;

  auto smart = std::make_unique<SmartArtillery>(config, gConfig, cLimits);

  StateVector state = smart->getState();
  state.position = Eigen::Vector3d(0, 0, 1000.0);
  state.velocity = Eigen::Vector3d(0, 0, -10.0); // Falling
  smart->setState(state);

  world.addProjectile(std::move(smart));

  // Step
  for (int i = 0; i < 100; ++i) {
    world.step(0.1);
    auto p = world.getProjectiles()[0].get();
    if (i % 10 == 0) {
      std::cout << "t=" << p->getState().elapsedTime
                << " z=" << p->getState().position.z()
                << " vz=" << p->getState().velocity.z()
                << " landed=" << p->hasLanded() << std::endl;
    }
    if (p->hasLanded()) {
      std::cout << "Landed at t=" << p->getState().elapsedTime << std::endl;
      break;
    }
  }
}

void test_smart_guidance_phases() {
  std::cout << "\n--- Guidance Phase Test ---" << std::endl;
  SimulationWorld world;
  LaunchConfig config;
  config.elevation_deg = 45; // Launch at 45
  config.dryMass_kg = 50;
  config.fuelMass_kg = 10;
  config.thrust_N = 5000;      // High thrust
  config.burnDuration_s = 2.0; // Short burn (Boost phase)
  config.massFlowRate_kgps = 5;
  config.referenceArea_m2 = 0.01;

  GuidanceConfig gConfig;
  ControlLimits cLimits;

  auto smart = std::make_unique<SmartArtillery>(config, gConfig, cLimits);
  smart->setGuidanceEnabled(true);

  world.addProjectile(std::move(smart));

  // Run simulation
  bool reachedBallistic = false;
  bool reachedTerminal = false;

  for (int i = 0; i < 2000; ++i) { // 200 seconds max
    double dt = 0.1;
    world.step(dt);
    const auto *p =
        static_cast<const SmartArtillery *>(world.getProjectiles()[0].get());
    auto phase = p->getPhase();

    if (phase == SmartArtillery::GuidancePhase::BALLISTIC)
      reachedBallistic = true;
    if (phase == SmartArtillery::GuidancePhase::TERMINAL)
      reachedTerminal = true;

    if (p->hasLanded()) {
      std::cout << "Landed at x=" << p->getState().position.x() << std::endl;
      break;
    }

    if (i % 50 == 0) { // Log every 5 sec
      std::cout << "t=" << std::fixed << std::setprecision(1)
                << p->getState().elapsedTime
                << " z=" << p->getState().position.z()
                << " Phase=" << (int)phase << std::endl;
    }
  }

  if (reachedBallistic)
    std::cout << "[PASS] Reached Ballistic Phase" << std::endl;
  else
    std::cout << "[FAIL] Missed Ballistic Phase" << std::endl;

  if (reachedTerminal)
    std::cout << "[PASS] Reached Terminal Phase" << std::endl;
  else
    std::cout << "[FAIL] Missed Terminal Phase" << std::endl;
}

int main() {
  test_drop_physics();
  test_smart_guidance_phases();
  return 0;
}
