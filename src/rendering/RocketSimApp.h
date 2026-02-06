#pragma once

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include "../guidance/ControlSystem.h" // For ControlLimits
#include "../guidance/GuidanceConfig.h"
#include "../simulation/SimulationWorld.h"
#include "../utils/LaunchConfig.h"
#include "Camera.h"
#include "Renderer.h"

#include "imgui.h"
#include "implot.h"

#include <vector>

class RocketSimApp {
public:
  RocketSimApp();
  ~RocketSimApp();

  // Initialize window, OpenGL, ImGui
  bool init();

  // Run the main loop
  void run();

private:
  // Core loop steps
  void processInput();
  void updatePhysics();
  void render();
  void renderUI();

  // Helper for UI panels
  void renderLaunchControls();
  void renderTelemetry();
  void renderAnalysis();

  // Application State
  GLFWwindow *m_window = nullptr;
  SimulationWorld m_world;
  Renderer m_renderer;
  Camera m_camera;

  // Simulation Configuration & State
  LaunchConfig m_config;
  GuidanceConfig m_guidanceConfig;
  ControlLimits m_controlLimits;

  // Launch Mode
  enum class LaunchMode { DUMB, SMART };
  LaunchMode m_launchMode = LaunchMode::DUMB;

  // Simulation Runtime State
  bool m_simulationRunning = false;
  double m_accumulatedTime = 0.0;
  float m_timeScale = 1.0f;
  bool m_autoZoom = true;
  double m_maxDist = 50.0;

  // Smart Artillery Targets
  float m_targetX = 5000.0f;
  float m_targetY = 0.0f;
  float m_targetZ = 0.0f;
  double m_calculatedElevation = 45.0;
  double m_calculatedRange = 0.0;
  bool m_isTargetReachable = true;
  bool m_sensorNoiseEnabled = false;

  // Wind Configuration
  float m_windSpeed = 0.0f;
  float m_windDirection = 0.0f;

  // UI Caching for Plots
  struct PlotCache {
    std::vector<std::vector<double>> altitude_x;
    std::vector<std::vector<double>> altitude_y;
    std::vector<std::vector<double>> velocity_t;
    std::vector<std::vector<double>> velocity_v;
    std::vector<std::vector<double>> mach_alt;
    std::vector<std::vector<double>> mach_mach;
    std::vector<std::vector<double>> mass_t;
    std::vector<std::vector<double>> mass_m;

    void clear() {
      altitude_x.clear();
      altitude_y.clear();
      velocity_t.clear();
      velocity_v.clear();
      mach_alt.clear();
      mach_mach.clear();
      mass_t.clear();
      mass_m.clear();
    }
  } m_plotCache;

  int m_plotUpdateCounter = 0;
};
