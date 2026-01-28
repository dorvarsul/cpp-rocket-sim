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

void SmartArtillery::update(double dt) {
  // This gets called by SimulationWorld
  // The actual physics integration happens there via RK4
  // We just need to make sure our state is up to date
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

Eigen::Vector3d
SmartArtillery::computeFinLift(const StateVector &state,
                               const Eigen::Vector3d &windVelocity) {
  // Return zero if guidance is disabled or projectile has landed
  if (!m_guidanceEnabled || m_landed) {
    return Eigen::Vector3d::Zero();
  }

  // Don't apply guidance until sufficient velocity (boost phase)
  // This prevents wild overcorrections when rocket is still accelerating
  const double MIN_GUIDANCE_VELOCITY = 50.0; // m/s
  double currentSpeed = state.velocity.norm();
  if (currentSpeed < MIN_GUIDANCE_VELOCITY) {
    return Eigen::Vector3d::Zero();
  }

  // Get estimated state from navigation system
  StateVector estimatedState = m_navigation.getEstimatedState(state);

  // Compute guidance command (desired lateral acceleration)
  Eigen::Vector3d guidanceAccel =
      m_guidance.computeGuidanceCommand(estimatedState);

  // Sprint 3: Wind Correction
  // Aerodynamic forces depend on Relative Velocity (Airspeed)
  Eigen::Vector3d relativeVelocity = state.velocity - windVelocity;
  double airspeed = relativeVelocity.norm();

  // Compute dynamic pressure using Airspeed
  double altitude = state.position.z();
  double rho = m_atmosphere.getDensity(altitude);
  double dynamicPressure = 0.5 * rho * airspeed * airspeed;

  // Translate guidance command to fin deflections
  // Note: Control system should technically operate on airspeed too for scaling
  Eigen::Vector2d finAngles = m_control.computeFinDeflection(
      guidanceAccel, relativeVelocity, dynamicPressure);

  // Compute aerodynamic lift force from fins using Airspeed
  Eigen::Vector3d finLift = m_control.computeFinLift(
      finAngles, relativeVelocity, dynamicPressure, m_aero.getReferenceArea());

  return finLift;
}

void SmartArtillery::setCurrentGLoad(const Eigen::Vector3d &totalAcceleration) {
  m_currentGLoad = m_control.getCurrentGLoad(totalAcceleration);
}

Eigen::Vector3d SmartArtillery::computeTotalForces(double elapsedTime) {
  // Get basic forces from parent (gravity, drag, thrust)
  Eigen::Vector3d totalForce = Eigen::Vector3d::Zero();

  // Gravity
  const double g = 9.81;
  totalForce += Eigen::Vector3d(0, 0, -g * m_state.totalMass);

  // Drag
  totalForce += getDragForce();

  // Thrust
  totalForce += getThrustForce();

  // Add guidance forces if enabled
  if (m_guidanceEnabled && !m_landed) {
    // Get estimated state from navigation system
    StateVector estimatedState = m_navigation.getEstimatedState(m_state);

    // Compute guidance command
    Eigen::Vector3d guidanceAccel =
        m_guidance.computeGuidanceCommand(estimatedState);

    // Compute dynamic pressure
    double altitude = m_state.position.z();
    double rho = m_atmosphere.getDensity(altitude);
    double speed = m_state.velocity.norm();
    double dynamicPressure = 0.5 * rho * speed * speed;

    // Translate guidance to fin deflections
    Eigen::Vector2d finAngles = m_control.computeFinDeflection(
        guidanceAccel, m_state.velocity, dynamicPressure);

    // Compute aerodynamic lift from fins
    Eigen::Vector3d finLift =
        m_control.computeFinLift(finAngles, m_state.velocity, dynamicPressure,
                                 m_aero.getReferenceArea());

    totalForce += finLift;

    // Calculate G-load for telemetry
    Eigen::Vector3d totalAccel = totalForce / m_state.totalMass;
    m_currentGLoad = m_control.getCurrentGLoad(totalAccel);
  }

  return totalForce;
}
