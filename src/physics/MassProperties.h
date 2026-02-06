#pragma once

/**
 * Mass Properties
 * Tracks rocket mass breakdown (dry mass + fuel)
 */
class MassProperties {
public:
  /**
   * Constructor
   * @param dryMass_kg Dry mass without fuel (kg)
   * @param fuelMass_kg Initial fuel mass (kg)
   */
  MassProperties(double dryMass_kg, double fuelMass_kg);

  /**
   * Get total mass (dry + fuel)
   * @return Total mass in kg
   */
  double getTotalMass() const;

  /**
   * Get dry mass only
   * @return Dry mass in kg
   */
  double getDryMass() const;

  /**
   * Get remaining fuel mass
   * @return Fuel mass in kg
   */
  double getRemainingFuel() const;

  /**
   * Consume fuel (decreases fuel mass)
   * @param amount_kg Amount of fuel to consume (kg)
   */
  void consumeFuel(double amount_kg);

  /**
   * Set remaining fuel directly
   * @param fuelMass_kg New fuel mass (kg)
   */
  void setRemainingFuel(double fuelMass_kg);

private:
  double m_dryMass;
  double m_fuelMass;
};
