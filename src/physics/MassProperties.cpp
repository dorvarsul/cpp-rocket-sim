#include "MassProperties.h"
#include <algorithm>

MassProperties::MassProperties(double dryMass_kg, double fuelMass_kg)
    : m_dryMass(dryMass_kg), m_fuelMass(fuelMass_kg) {}

double MassProperties::getTotalMass() const { return m_dryMass + m_fuelMass; }

double MassProperties::getDryMass() const { return m_dryMass; }

double MassProperties::getRemainingFuel() const { return m_fuelMass; }

void MassProperties::consumeFuel(double amount_kg) {
  m_fuelMass = std::max(0.0, m_fuelMass - amount_kg);
}

void MassProperties::setRemainingFuel(double fuelMass_kg) {
  m_fuelMass = std::max(0.0, fuelMass_kg);
}
