#pragma once

struct LaunchConfig {
  double elevation_deg; // Angle from horizontal (0-90 degrees)
  double azimuth_deg;   // Angle from East, Counter-Clockwise (0=East, 90=North)
  double muzzle_velocity_mps; // Initial velocity magnitude in m/s
};
