#include "SmartArtillery.h"
#include "AtmosphereModel.h"

SmartArtillery::SmartArtillery(const LaunchConfig &config,
                               const GuidanceConfig &guidanceConfig,
                               const ControlLimits &controlLimits,
                               bool sensorNoiseEnabled)
    : DumbArtillery(config), m_navigation(), m_guidance(guidanceConfig, config),
      m_control(controlLimits) {
  m_navigation.setSensorNoise(sensorNoiseEnabled);
}

double SmartArtillery::getDistanceToTarget() const {
  return m_guidance.getDistanceToTarget(m_state.position);
}

bool SmartArtillery::hasReachedTarget() const {
  return m_guidance.hasReachedTarget(m_state.position);
}

Eigen::Vector2d SmartArtillery::getFinAngles() const {
  return m_control.getFinAngles();
}

void SmartArtillery::setTarget(const Eigen::Vector3d &newTarget) {
  m_guidance.setTarget(newTarget);
}

void SmartArtillery::setCurrentGLoad(const Eigen::Vector3d &totalAcceleration) {
  m_currentGLoad = m_control.getCurrentGLoad(totalAcceleration);
}

void SmartArtillery::updateTelemetry(const Eigen::Vector3d &acceleration) {
  // Update Phase
  double time = m_state.elapsedTime;

  switch (m_phase) {
  case GuidancePhase::IDLE:
    if (time > 0.0 && m_propulsion.isBurning(time)) {
      m_phase = GuidancePhase::ASCENT;
    }
    break;
  case GuidancePhase::ASCENT:
    if (!m_propulsion.isBurning(time)) {
      m_phase = GuidancePhase::BALLISTIC;
    }
    break;
  case GuidancePhase::BALLISTIC:
    // Detect Apogee (vertical velocity turns negative)
    if (m_state.velocity.z() < 0.0) {
      m_phase = GuidancePhase::TERMINAL;
    }
    break;
  case GuidancePhase::TERMINAL:
    if (m_landed) {
      m_phase = GuidancePhase::IDLE; // Reset? Or stay terminal/ended
    }
    break;
  }

  // Base telemetry
  setCurrentGLoad(acceleration);
}

void SmartArtillery::update(double dt) {
  // Unused
}

Eigen::Vector3d
SmartArtillery::computeForces(const StateVector &state,
                              const Eigen::Vector3d &windVelocity) const {
  // 1. Get base forces (Gravity + Drag + Thrust)
  Eigen::Vector3d totalForce =
      DumbArtillery::computeForces(state, windVelocity);

  // 2. Apply Guidance ONLY in TERMINAL phase
  if (!m_guidanceEnabled || m_phase != GuidancePhase::TERMINAL) {
    return totalForce;
  }

  // Extra safety: Don't guide if too slow (aerodynamically ineffective)
  if (state.velocity.norm() < 30.0) {
    return totalForce;
  }

  // Get estimated state from navigation system
  auto &nav = const_cast<NavigationSystem &>(m_navigation);
  StateVector estimatedState = nav.getEstimatedState(state);

  // Compute guidance command
  auto &guid = const_cast<GuidanceUnit &>(m_guidance);
  Eigen::Vector3d guidanceAccel = guid.computeGuidanceCommand(estimatedState);

  // Wind Correction for Control
  Eigen::Vector3d relativeVelocity = state.velocity - windVelocity;
  double airspeed = relativeVelocity.norm();

  // Dynamic Pressure
  double altitude = state.position.z();
  double rho = m_atmosphere.getDensity(altitude);
  double dynamicPressure = 0.5 * rho * airspeed * airspeed;

  // Fin Deflection
  auto &ctrl = const_cast<ControlSystem &>(m_control);
  Eigen::Vector2d finAngles = ctrl.computeFinDeflection(
      guidanceAccel, relativeVelocity, dynamicPressure);

  // Fin Lift
  Eigen::Vector3d finLift = ctrl.computeFinLift(
      finAngles, relativeVelocity, dynamicPressure, m_aero.getReferenceArea());

  return totalForce + finLift;
}
