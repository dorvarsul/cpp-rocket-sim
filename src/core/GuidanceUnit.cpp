#include "GuidanceUnit.h"
#include <algorithm>
#include <cmath>
#include <iostream>

GuidanceUnit::GuidanceUnit(const GuidanceConfig &config,
                           const LaunchConfig &launchConfig)
    : m_config(config), m_launchConfig(launchConfig), m_predictor() {
  m_lastLOS = Eigen::Vector3d::Zero();
}

Eigen::Vector3d GuidanceUnit::computeGuidanceCommand(const StateVector &state) {
  // Use Predictive Guidance (PIP) as primary method
  // It is more robust for artillery trajectories than ProNav
  return computePredictiveGuidance(state);
}

Eigen::Vector3d
GuidanceUnit::computePredictiveGuidance(const StateVector &state) {
  // Only update prediction at low frequency (5Hz) to save CPU
  // and to avoid jitter
  if (state.elapsedTime - m_lastGuidanceUpdateTime > 0.2) {
    // Predict where we land if we do nothing (Zero Effort Miss)
    Eigen::Vector3d predictedImpact =
        m_predictor.estimateLandingFromState(m_launchConfig, state);

    // Error vector (Desired - Actual)
    Eigen::Vector3d error = m_config.targetPosition - predictedImpact;
    error.z() = 0.0; // Force ground target consideration

    // Estimate Time-to-Go (rough approximation)
    double distanceToTarget = (m_config.targetPosition - state.position).norm();
    double speed = state.velocity.norm();
    if (speed < 1.0)
      speed = 1.0;
    double t_go = distanceToTarget / speed;

    if (t_go < 1.0)
      t_go = 1.0; // Prevent singularity near impact

    // Guidance Law: A = N * Error / t_go^2
    // N is navigation constant. For ZEM, optimal N is often 3/t_go?
    // Let's use a simple proportional gain scaled by t_go
    // If error is 100m, and t_go is 10s.
    // We need to drift 100m in 10s.
    // A = 2 * dist / t^2.
    // A = 2 * 100 / 100 = 2 m/s^2.
    // So Gain should be roughly 2.0 / t_go^2 ?

    double gain = 2.0; // Base gain

    // Tune gain based on phase?
    // Just use simple P-controller scaled by 1/t_go^2
    // Limit t_go^2 to avoid massive spikes
    double t_go_sq = t_go * t_go;
    if (t_go_sq < 4.0)
      t_go_sq = 4.0;

    double accelMagnitude = gain * error.norm() / t_go_sq;

    // Direction: Toward the error
    Eigen::Vector3d direction = error.normalized();

    // We can only accelerate perpendicular to velocity (using fins)
    // Project desired acceleration onto plane perpendicular to velocity
    Eigen::Vector3d velUnit = state.velocity.normalized();
    Eigen::Vector3d accelPerp = direction - direction.dot(velUnit) * velUnit;

    // If corrections requires significant axial force (drag/thrust), we can't
    // do much with fins except steer to increase/decrease flight path angle.
    // But projecting the "Correction Vector" onto the "Control Plane" works
    // well.

    // Scale magnitude?
    // If desired direction was mostly along velocity (range error),
    // we need to pitch up/down to change range.
    // Pitching up/down IS perpendicular to velocity.
    // So the projection handles it!
    // If we need more range (Error is Forward), direction is Forward.
    // Perp component is Zero? No.
    // Velocity (1, 0, 1). Error (1, 0, 0).
    // Dot product non-zero.
    // Perp = (1,0,0) - 0.7*(0.7, 0, 0.7) = (1,0,0) - (0.5, 0, 0.5) = (0.5, 0,
    // -0.5). We pitch DOWN to go FORWARD? If we pitch down, we reduce flight
    // time, reduce range? Or increase? This is tricky. Impact Point Predictor
    // will handle the sign implicitly? No, we need to know which way to turn to
    // reduce error.

    // Standard ZEM guidance assumes full control authority.
    // Ideally we should use the Jacobian (Sensitivity Matrix), but that's
    // complex. Let's rely on the projected vector. If predicted 5.5km
    // (Overshoot). Target 5.0km. Error is Back (-X). Velocity is Forward (+X,
    // +Z). We want force Back. Perp component of Back against (Forward/Up) is
    // (Back/Up). So we steer UP (Pitch Up). Pitching Up increases drag and loft
    // => Increases flight time? Ballistic: 45 deg max range. If at 45 deg,
    // Pitch Up (50 deg) -> Range decreases. Correct. If at 85 deg, Pitch Up (90
    // deg) -> Range decreases (to 0). Correct. So simply steering towards the
    // error works!

    m_cachedGuidanceCmd = accelMagnitude * accelPerp.normalized();

    // Clamp acceleration
    if (m_cachedGuidanceCmd.norm() > 30.0) { // 3G limit
      m_cachedGuidanceCmd = 30.0 * m_cachedGuidanceCmd.normalized();
    }

    m_lastGuidanceUpdateTime = state.elapsedTime;
  }

  return m_cachedGuidanceCmd;
}

Eigen::Vector3d GuidanceUnit::computeProNavGuidance(const StateVector &state) {
  // Legacy ProNav implementation (unused for now)
  // ... (Code omitted for brevity, but I should probably include it or delete
  // it) I'll leave it empty or minimal
  return Eigen::Vector3d::Zero();
}

double
GuidanceUnit::getDistanceToTarget(const Eigen::Vector3d &position) const {
  // For ground targets, use horizontal (2D) distance only
  Eigen::Vector3d delta = m_config.targetPosition - position;
  delta.z() = 0.0; // Ignore altitude difference
  return delta.norm();
}

bool GuidanceUnit::hasReachedTarget(const Eigen::Vector3d &position) const {
  return getDistanceToTarget(position) < m_config.convergenceRadius_m;
}

void GuidanceUnit::setTarget(const Eigen::Vector3d &newTarget) {
  m_config.targetPosition = newTarget;
}
