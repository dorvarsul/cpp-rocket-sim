#include "AtmosphereModel.h"
#include <cmath>

AtmosphereModel::AtmosphereModel() {}

double AtmosphereModel::getTemperature(double altitude_m) const {
  if (altitude_m <= TROPO_ALT) {
    // Troposphere: linear temperature decrease
    return T0 + LAPSE_RATE * altitude_m;
  } else {
    // Stratosphere: constant temperature
    return T0 + LAPSE_RATE * TROPO_ALT;
  }
}

double AtmosphereModel::getPressure(double altitude_m) const {
  if (altitude_m <= TROPO_ALT) {
    // Troposphere
    double T = getTemperature(altitude_m);
    double exponent = -g / (LAPSE_RATE * R);
    return P0 * std::pow(T / T0, exponent);
  } else {
    // Stratosphere: exponential decrease
    double P_tropo = getPressure(TROPO_ALT);
    double T_tropo = getTemperature(TROPO_ALT);
    double exponent = -g * (altitude_m - TROPO_ALT) / (R * T_tropo);
    return P_tropo * std::exp(exponent);
  }
}

double AtmosphereModel::getDensity(double altitude_m) const {
  double P = getPressure(altitude_m);
  double T = getTemperature(altitude_m);
  // Ideal gas law: ρ = P / (R * T)
  return P / (R * T);
}

double AtmosphereModel::getSpeedOfSound(double altitude_m) const {
  double T = getTemperature(altitude_m);
  // Speed of sound: a = √(γ * R * T)
  return std::sqrt(GAMMA * R * T);
}
