#pragma once

/**
 * International Standard Atmosphere (ISA) Model
 * Provides altitude-dependent atmospheric properties
 */
class AtmosphereModel {
public:
  AtmosphereModel();

  /**
   * Get air density at given altitude
   * @param altitude_m Altitude above sea level in meters
   * @return Air density in kg/m³
   */
  double getDensity(double altitude_m) const;

  /**
   * Get temperature at given altitude
   * @param altitude_m Altitude above sea level in meters
   * @return Temperature in Kelvin
   */
  double getTemperature(double altitude_m) const;

  /**
   * Get pressure at given altitude
   * @param altitude_m Altitude above sea level in meters
   * @return Pressure in Pascals
   */
  double getPressure(double altitude_m) const;

  /**
   * Get speed of sound at given altitude
   * @param altitude_m Altitude above sea level in meters
   * @return Speed of sound in m/s
   */
  double getSpeedOfSound(double altitude_m) const;

private:
  // Sea level constants
  static constexpr double T0 = 288.15;   // Sea level temperature (K)
  static constexpr double P0 = 101325.0; // Sea level pressure (Pa)
  static constexpr double RHO0 = 1.225;  // Sea level density (kg/m³)
  static constexpr double g = 9.80665;   // Gravitational acceleration (m/s²)
  static constexpr double R =
      287.05; // Specific gas constant for air (J/(kg·K))
  static constexpr double GAMMA = 1.4;          // Ratio of specific heats
  static constexpr double LAPSE_RATE = -0.0065; // Temperature lapse rate (K/m)
  static constexpr double TROPO_ALT = 11000.0;  // Tropopause altitude (m)
};
