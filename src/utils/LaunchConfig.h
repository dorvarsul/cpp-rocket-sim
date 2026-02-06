#pragma once

struct LaunchConfig {
  // Sprint 1: Ballistics
  double elevation_deg; // Angle from horizontal (0-90 degrees)
  double azimuth_deg;   // Angle from East, Counter-Clockwise (0=East, 90=North)

  // Sprint 2: Mass properties
  double dryMass_kg = 50.0;  // Dry mass (structure, payload) in kg
  double fuelMass_kg = 10.0; // Initial fuel mass in kg

  // Sprint 2: Aerodynamics
  double referenceArea_m2 = 0.01; // Cross-sectional area in m²

  // Sprint 2: Propulsion
  double thrust_N = 0.0;          // Thrust force in Newtons
  double burnDuration_s = 0.0;    // Burn duration in seconds
  double massFlowRate_kgps = 0.0; // Fuel consumption rate in kg/s
};
