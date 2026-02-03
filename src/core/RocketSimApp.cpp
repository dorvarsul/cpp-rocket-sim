#include "RocketSimApp.h"
#include "core/DumbArtillery.h"
#include "core/LaunchAngleSolver.h"
#include "core/SmartArtillery.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "imgui_internal.h"

#include <cmath>
#include <iostream>

// Helper for GLFW callbacks
static RocketSimApp *g_AppInstance = nullptr;

static void scroll_callback(GLFWwindow *window, double xoffset,
                            double yoffset) {
  if (g_AppInstance && yoffset > 0) {
    // Access camera via app instance if possible or just use logic
    // Ideally App should expose camera control
    // For now, let's just use the logic from main.cpp but slightly adjusted or
    // rely on app processing it.
    // But main.cpp logic was: camera.setDistance(50.0f);
    // We need access to camera.
    // Let's skipping implementing simple scroll callback for now or put logic
    // in processInput actually main.cpp had scroll_callback.
  }
}

RocketSimApp::RocketSimApp() {
  g_AppInstance = this;

  // Initialize Config Defaults (Sprint 2 / main.cpp defaults)
  m_config.elevation_deg = 45.0;
  m_config.azimuth_deg = 0.0;
  m_config.dryMass_kg = 100.0;
  m_config.fuelMass_kg = 50.0;
  m_config.referenceArea_m2 = 0.02;
  m_config.thrust_N = 5000.0;
  m_config.burnDuration_s = 15.0;
  m_config.massFlowRate_kgps = 50.0 / 15.0;

  // Guidance Defaults
  m_guidanceConfig.proNavGain = 2.0;
  m_guidanceConfig.convergenceRadius_m = 10.0;

  // Control Limits
  m_controlLimits.maxFinDeflection_deg = 20.0;
  m_controlLimits.maxGLoad = 10.0;
}

RocketSimApp::~RocketSimApp() {
  // Cleanup
  if (m_window) {
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();

    glfwDestroyWindow(m_window);
    glfwTerminate();
  }
  g_AppInstance = nullptr;
}

bool RocketSimApp::init() {
  if (!glfwInit())
    return false;

  const char *glsl_version = "#version 330";
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

  m_window = glfwCreateWindow(1280, 720, "Rocket Sim - Sprint 2", NULL, NULL);
  if (!m_window)
    return false;

  glfwMakeContextCurrent(m_window);
  glfwSwapInterval(1); // Enable vsync

  glewExperimental = GL_TRUE;
  if (glewInit() != GLEW_OK)
    return false;

  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND); // Ensure blending is enabled if used
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  // Setup Dear ImGui context
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImPlot::CreateContext();
  ImGuiIO &io = ImGui::GetIO();
  (void)io;
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
  io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;

  ImGui::StyleColorsDark();

  ImGui_ImplGlfw_InitForOpenGL(m_window, true);
  ImGui_ImplOpenGL3_Init(glsl_version);

  // Register scroll callback if needed, or rely on ImGui
  // glfwSetScrollCallback(m_window, scroll_callback);

  // Renderer Init
  m_renderer.init();

  return true;
}

void RocketSimApp::run() {
  while (!glfwWindowShouldClose(m_window)) {
    glfwPollEvents();

    processInput();
    updatePhysics();
    render(); // This calls renderUI inside
  }
}

void RocketSimApp::processInput() {
  // Manual Camera Input
  m_camera.processInput(m_window);
}

void RocketSimApp::updatePhysics() {
  // Fixed time step recommended
  constexpr double PHYSICS_DT = 0.01;

  if (m_simulationRunning) {
    static double lastFrameTime = glfwGetTime();
    double currentFrameTime = glfwGetTime();
    double frameDelta = currentFrameTime - lastFrameTime;
    lastFrameTime = currentFrameTime;

    // Prevent huge delta steps (spiral of death)
    if (frameDelta > 0.1)
      frameDelta = 0.1;

    m_accumulatedTime += frameDelta * m_timeScale;

    while (m_accumulatedTime >= PHYSICS_DT) {
      m_world.step(PHYSICS_DT);
      m_accumulatedTime -= PHYSICS_DT;
    }
  } else {
    // Keep timer updated to avoid jump on resume
    static double lastFrameTime = glfwGetTime();
    lastFrameTime = glfwGetTime();
  }

  // Auto-zoom logic
  if (m_autoZoom) {
    bool hasData = false;
    for (const auto &p : m_world.getProjectiles()) {
      const auto &history = p->getHistory();
      if (!history.empty()) {
        hasData = true;
        for (const auto &s : history) {
          double d = s.position.norm();
          if (d > m_maxDist)
            m_maxDist = d;
        }
      }
    }
    if (hasData) {
      m_camera.setDistance((float)(m_maxDist * 1.5));
    }
  }
  m_camera.update();
}

void RocketSimApp::render() {
  // Start ImGui Frame
  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();

  renderUI(); // Dockspace and panels

  ImGui::Render();

  int display_w, display_h;
  glfwGetFramebufferSize(m_window, &display_w, &display_h);
  glViewport(0, 0, display_w, display_h);

  glClearColor(0.05f, 0.05f, 0.08f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // Render 3D Scene
  Eigen::Matrix4f view = m_camera.getViewMatrix();
  Eigen::Matrix4f proj =
      m_camera.getProjectionMatrix((float)display_w / (float)display_h);

  std::vector<std::vector<StateVector>> allTrajectories;
  for (const auto &p : m_world.getProjectiles()) {
    allTrajectories.push_back(p->getHistory());
  }

  m_renderer.render(view, proj, allTrajectories, m_camera.getDistance());

  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
  glfwSwapBuffers(m_window);
}

void RocketSimApp::renderUI() {
  // Dockspace
  ImGuiID dockSpaceId = ImGui::GetID("RocketSimDockSpace");
  ImGui::DockSpaceOverViewport(dockSpaceId, ImGui::GetMainViewport(),
                               ImGuiDockNodeFlags_PassthruCentralNode);

  static bool isFirstRun = true;
  if (isFirstRun) {
    isFirstRun = false;
    ImGui::DockBuilderRemoveNode(dockSpaceId);
    ImGui::DockBuilderAddNode(dockSpaceId,
                              ImGuiDockNodeFlags_PassthruCentralNode |
                                  ImGuiDockNodeFlags_DockSpace);
    ImGui::DockBuilderSetNodeSize(dockSpaceId, ImGui::GetMainViewport()->Size);

    ImGuiID dockMain = dockSpaceId;
    ImGuiID dockLeft = ImGui::DockBuilderSplitNode(dockMain, ImGuiDir_Left,
                                                   0.20f, NULL, &dockMain);
    ImGuiID dockRight = ImGui::DockBuilderSplitNode(dockMain, ImGuiDir_Right,
                                                    0.25f, NULL, &dockMain);
    ImGuiID dockBottom = ImGui::DockBuilderSplitNode(dockMain, ImGuiDir_Down,
                                                     0.30f, NULL, &dockMain);

    ImGui::DockBuilderDockWindow("Launch Controls", dockLeft);
    ImGui::DockBuilderDockWindow("Telemetry Analysis", dockRight);
    ImGui::DockBuilderDockWindow("Live Telemetry", dockBottom);

    ImGui::DockBuilderFinish(dockSpaceId);
  }

  renderLaunchControls();
  renderTelemetry();
  renderAnalysis();
}

void RocketSimApp::renderLaunchControls() {
  ImGui::Begin("Launch Controls");
  ImGui::Checkbox("Auto-Zoom 3D", &m_autoZoom);
  ImGui::Separator();

  ImGui::Text("Preset Configurations");
  const char *presets[] = {"Custom", "Short-Range Mortar (2.5-8km)",
                           "Mid-Range Rocket (10-40km)"};
  static int currentPreset = 0;
  if (ImGui::Combo("##presets", &currentPreset, presets,
                   IM_ARRAYSIZE(presets))) {
    if (currentPreset == 1) {
      m_config.elevation_deg = 45.0;
      m_config.azimuth_deg = 0.0;
      m_config.dryMass_kg = 8.3;
      m_config.fuelMass_kg = 3.2;
      m_config.referenceArea_m2 = 0.01;
      m_config.thrust_N = 6200.0;
      m_config.burnDuration_s = 1.1;
      m_config.massFlowRate_kgps = 2.9;
    } else if (currentPreset == 2) {
      m_config.elevation_deg = 45.0;
      m_config.azimuth_deg = 0.0;
      m_config.dryMass_kg = 45.5;
      m_config.fuelMass_kg = 20.5;
      m_config.referenceArea_m2 = 0.025;
      m_config.thrust_N = 24000.0;
      m_config.burnDuration_s = 2.0;
      m_config.massFlowRate_kgps = 10.5;
    }
  }
  ImGui::Separator();

  ImGui::Text("Launch Mode");
  if (ImGui::RadioButton("Dumb Artillery", m_launchMode == LaunchMode::DUMB))
    m_launchMode = LaunchMode::DUMB;
  ImGui::SameLine();
  if (ImGui::RadioButton("Smart Artillery", m_launchMode == LaunchMode::SMART))
    m_launchMode = LaunchMode::SMART;
  ImGui::Separator();

  if (m_launchMode == LaunchMode::DUMB) {
    float el = (float)m_config.elevation_deg;
    if (ImGui::SliderFloat("Elevation (deg)", &el, 0.0f, 90.0f))
      m_config.elevation_deg = (double)el;
    float az = (float)m_config.azimuth_deg;
    if (ImGui::SliderFloat("Azimuth (deg)", &az, 0.0f, 360.0f))
      m_config.azimuth_deg = (double)az;
  } else {
    ImGui::Text("Target Coordinates (ground)");
    static float prevTargetX = m_targetX;
    static float prevTargetY = m_targetY;
    static LaunchConfig prevConfig = m_config;

    ImGui::InputFloat("Target X", &m_targetX, 100.0f, 1000.0f, "%.1f");
    ImGui::InputFloat("Target Y", &m_targetY, 100.0f, 1000.0f, "%.1f");
    m_targetZ = 0.0f;

    bool configChanged =
        (prevConfig.dryMass_kg != m_config.dryMass_kg ||
         prevConfig.fuelMass_kg != m_config.fuelMass_kg ||
         prevConfig.thrust_N != m_config.thrust_N ||
         prevConfig.burnDuration_s != m_config.burnDuration_s ||
         prevConfig.massFlowRate_kgps != m_config.massFlowRate_kgps ||
         prevConfig.referenceArea_m2 != m_config.referenceArea_m2);
    bool targetChanged = (prevTargetX != m_targetX || prevTargetY != m_targetY);

    if (targetChanged || configChanged) {
      // Reset state but don't auto-calculate to avoid lag while dragging
      m_isTargetReachable = false;
      m_calculatedElevation = 0.0;
      prevTargetX = m_targetX;
      prevTargetY = m_targetY;
      prevConfig = m_config;
    }

    if (ImGui::Button("CALCULATE SOLUTION")) {
      Eigen::Vector3d targetPos(m_targetX, m_targetY, 0.0);
      LaunchAngleSolver solver;

      // Always calculate firing solution
      auto [el, az] = solver.calculateLaunchAngles(m_config, targetPos);
      m_calculatedElevation = el;
      // Note: we don't automatically overwrite m_config.azimuth_deg unless in
      // Smart Mode strictly? Actually usually firing solution implies we should
      // set them. But let's just show the calculated values.

      m_calculatedRange = solver.getCalculatedRange();

      // Check reachability with a loose tolerance for UI display
      // If we got a valid range > 0, we show it.
      if (m_calculatedRange > 1.0) {
        m_isTargetReachable = true;
      } else {
        m_isTargetReachable = false;
      }
    }

    ImGui::Separator();
    if (m_isTargetReachable) {
      ImGui::Text("Calculated Launch Parameters:");
      ImGui::Text("  Elevation: %.2f deg", m_calculatedElevation * 0.75);
      ImGui::Text("  Azimuth: %.2f deg", m_config.azimuth_deg);
      double actualRange =
          std::sqrt(m_targetX * m_targetX + m_targetY * m_targetY);
      ImGui::Text("  Target Range: %.1f m", actualRange);
    } else {
      ImGui::TextColored(ImVec4(1, 0, 0, 1), "TARGET UNREACHABLE!");
    }

    ImGui::Separator();
    ImGui::Text("Guidance Parameters");
    float gain = (float)m_guidanceConfig.proNavGain;
    if (ImGui::SliderFloat("ProNav Gain", &gain, 1.0f, 6.0f))
      m_guidanceConfig.proNavGain = gain;
    float conv = (float)m_guidanceConfig.convergenceRadius_m;
    if (ImGui::SliderFloat("Convergence", &conv, 1.0f, 50.0f))
      m_guidanceConfig.convergenceRadius_m = conv;
    ImGui::Checkbox("Sensor Noise", &m_sensorNoiseEnabled);
  }

  ImGui::Separator();
  ImGui::Text("Environment");
  if (ImGui::Button("Wind Configuration"))
    ImGui::OpenPopup("Wind Config");
  if (ImGui::BeginPopup("Wind Config")) {
    bool changed = false;
    if (ImGui::SliderFloat("Speed (m/s)", &m_windSpeed, 0.0f, 50.0f))
      changed = true;
    if (ImGui::SliderFloat("Direction (deg)", &m_windDirection, 0.0f, 360.0f))
      changed = true;
    if (changed) {
      double rad = (double)m_windDirection * M_PI / 180.0;
      m_world.setWindVelocity(Eigen::Vector3d(
          m_windSpeed * std::cos(rad), m_windSpeed * std::sin(rad), 0.0));
    }
    ImGui::EndPopup();
  }
  if (std::abs(m_windSpeed) > 0.1f)
    ImGui::TextColored(ImVec4(0.5f, 0.5f, 1, 1), "Wind: %.1f m/s @ %.0f deg",
                       m_windSpeed, m_windDirection);
  else
    ImGui::TextDisabled("Wind: Calm");

  ImGui::Separator();
  ImGui::Text("Mass & Propulsion");
  float dm = (float)m_config.dryMass_kg;
  if (ImGui::SliderFloat("Dry Mass", &dm, 0.0f, 1000.0f))
    m_config.dryMass_kg = dm;
  float fm = (float)m_config.fuelMass_kg;
  if (ImGui::SliderFloat("Fuel Mass", &fm, 0.0f, 1000.0f))
    m_config.fuelMass_kg = fm;
  float thrust = (float)m_config.thrust_N;
  if (ImGui::SliderFloat("Thrust", &thrust, 0.0f, 50000.0f))
    m_config.thrust_N = thrust;
  float burn = (float)m_config.burnDuration_s;
  if (ImGui::SliderFloat("Burn Time", &burn, 0.0f, 60.0f))
    m_config.burnDuration_s = burn;

  if (ImGui::Button("FIRE")) {
    if (m_launchMode == LaunchMode::SMART) {
      if (!m_isTargetReachable) {
        ImGui::OpenPopup("Unreachable");
      } else {
        Eigen::Vector3d targetPos(m_targetX, m_targetY, m_targetZ);
        LaunchAngleSolver solver;
        auto [el, az] = solver.calculateLaunchAngles(m_config, targetPos);
        m_config.elevation_deg = el;
        m_config.azimuth_deg = az;
        m_guidanceConfig.targetPosition = targetPos;

        auto projectile = std::make_unique<SmartArtillery>(
            m_config, m_guidanceConfig, m_controlLimits, m_sensorNoiseEnabled);
        m_world.addProjectile(std::move(projectile));
        m_simulationRunning = true;
      }
    } else {
      auto projectile = std::make_unique<DumbArtillery>(m_config);
      m_world.addProjectile(std::move(projectile));
      m_simulationRunning = true;
    }
  }

  if (ImGui::BeginPopup("Unreachable")) {
    ImGui::Text("Target Unreachable!");
    if (ImGui::Button("OK"))
      ImGui::CloseCurrentPopup();
    ImGui::EndPopup();
  }

  ImGui::SameLine();
  if (ImGui::Button("Reset World")) {
    m_world.clear();
    m_simulationRunning = false;
    m_plotCache.clear();
  }

  ImGui::Separator();
  ImGui::SliderFloat("Time Scale", &m_timeScale, 0.0f, 10.0f);
  ImGui::Text("Projectiles: %zu", m_world.getProjectiles().size());

  ImGui::End();
}

void RocketSimApp::renderTelemetry() {
  ImGui::Begin("Live Telemetry");
  const auto &projs = m_world.getProjectiles();
  int active = 0;
  for (const auto &p : projs)
    if (!p->hasLanded())
      active++;

  if (active == 0)
    ImGui::Text("No active projectiles.");
  else {
    for (size_t i = 0; i < projs.size(); ++i) {
      const auto &p = projs[i];
      if (p->hasLanded())
        continue;

      StateVector s = p->getState();
      double range = std::sqrt(s.position.x() * s.position.x() +
                               s.position.y() * s.position.y());

      if (ImGui::CollapsingHeader(
              (std::string("Projectile ") + std::to_string(i)).c_str(),
              ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Text("Time: %.2f s", p->getHistory().size() * 0.01);
        ImGui::Text("Pos: %.1f, %.1f, %.1f", s.position.x(), s.position.y(),
                    s.position.z());
        ImGui::Text("Range: %.1f m", range);
        ImGui::Text("Speed: %.1f m/s", s.velocity.norm());
        ImGui::Text("Mass: %.1f kg (Fuel: %.1f)", s.totalMass, s.fuelMass);

        auto *smart = dynamic_cast<SmartArtillery *>(p.get());
        if (smart) {
          ImGui::Separator();
          ImGui::Text("Guidance: %s",
                      smart->isGuidanceActive()
                          ? "ACTIVE"
                          : (smart->isGuidanceEnabled() ? "STANDBY" : "OFF"));
          ImGui::Text("Dist to Target: %.1f m", smart->getDistanceToTarget());
          ImGui::Text("G-Load: %.2f G", smart->getCurrentGLoad());
        }
      }
    }
  }
  ImGui::End();
}

void RocketSimApp::renderAnalysis() {
  ImGui::Begin("Telemetry Analysis");

  // Cache Logic
  if (m_plotUpdateCounter++ % 3 == 0) {
    m_plotCache.clear();
    for (const auto &p : m_world.getProjectiles()) {
      const auto &hist = p->getHistory();
      if (hist.empty())
        continue;

      std::vector<double> x, y, t, v;
      for (size_t k = 0; k < hist.size(); ++k) {
        const auto &s = hist[k];
        x.push_back(std::sqrt(s.position.x() * s.position.x() +
                              s.position.y() * s.position.y()));
        y.push_back(s.position.z());
        t.push_back(k * 0.01);
        v.push_back(s.velocity.norm());
      }
      m_plotCache.altitude_x.push_back(x);
      m_plotCache.altitude_y.push_back(y);
      m_plotCache.velocity_t.push_back(t);
      m_plotCache.velocity_v.push_back(v);
    }
  }

  if (ImPlot::BeginPlot("Altitude vs Range")) {
    ImPlot::SetupAxes("Range (m)", "Altitude (m)", ImPlotAxisFlags_AutoFit,
                      ImPlotAxisFlags_AutoFit);
    for (size_t i = 0; i < m_plotCache.altitude_x.size(); ++i) {
      ImPlot::PlotLine(
          ("P" + std::to_string(i)).c_str(), m_plotCache.altitude_x[i].data(),
          m_plotCache.altitude_y[i].data(), m_plotCache.altitude_x[i].size());
    }
    ImPlot::EndPlot();
  }

  if (ImPlot::BeginPlot("Velocity vs Time")) {
    ImPlot::SetupAxes("Time (s)", "Velocity (m/s)", ImPlotAxisFlags_AutoFit,
                      ImPlotAxisFlags_AutoFit);
    for (size_t i = 0; i < m_plotCache.velocity_t.size(); ++i) {
      ImPlot::PlotLine(
          ("P" + std::to_string(i)).c_str(), m_plotCache.velocity_t[i].data(),
          m_plotCache.velocity_v[i].data(), m_plotCache.velocity_t[i].size());
    }
    ImPlot::EndPlot();
  }
  ImGui::End();
}
