#pragma once
#include "IProjectile.h"
#include "LaunchConfig.h"
#include <vector>

class DumbArtillery : public IProjectile {
public:
  explicit DumbArtillery(const LaunchConfig &config);

  void update(double dt) override;
  StateVector getState() const override;
  std::vector<StateVector> getHistory() const override;
  bool hasLanded() const override;

  // Helper for RK4
  void setState(const StateVector &newState);

private:
  StateVector m_state;
  std::vector<StateVector> m_history;
  bool m_landed = false;
};
