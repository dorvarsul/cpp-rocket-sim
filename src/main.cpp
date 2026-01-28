#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "imgui_internal.h"
#include "implot.h"
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include "core/DumbArtillery.h"
#include "core/LaunchAngleSolver.h"
#include "core/SimulationWorld.h"
#include "core/SmartArtillery.h"
#include "visualization/Camera.h"
#include "visualization/Renderer.h"

#include <cmath>
#include <vector>

// Global Camera for input callback (simple approach)
Camera camera;

void scroll_callback(GLFWwindow *window, double xoffset, double yoffset) {
  (void)window;
  (void)xoffset;
  // Zoom
  if (yoffset > 0)
    camera.setDistance(50.0f); // Reset or better zoom logic?
                               // Let's implement incremental zoom
  // Since we don't have easy getter for distance, we rely on camera internal
  // state or simple logic
}

int main() {
  if (!glfwInit())
    return 1;

  const char *glsl_version = "#version 330";
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

  GLFWwindow *window =
      glfwCreateWindow(1280, 720, "Rocket Sim - Sprint 2", NULL, NULL);
  if (window == NULL)
    return 1;

  glfwMakeContextCurrent(window);
  glfwSwapInterval(1); // Enable vsync

  glewExperimental = GL_TRUE;
  if (glewInit() != GLEW_OK)
    return 1;

  glEnable(GL_DEPTH_TEST);

  // Setup Dear ImGui context
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImPlot::CreateContext();
  ImGuiIO &io = ImGui::GetIO();
  (void)io;
  io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
  io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;

  ImGui::StyleColorsDark();

  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL3_Init(glsl_version);

  // Simulation Setup
  SimulationWorld world;
  Renderer renderer;
  renderer.init();

  // Simulation State
  LaunchConfig config;
  config.elevation_deg = 45.0;
  config.azimuth_deg = 0.0;
  config.dryMass_kg = 100.0;
  config.fuelMass_kg = 50.0;
  config.referenceArea_m2 = 0.02;
  config.thrust_N = 5000.0;
  config.burnDuration_s = 15.0;
  config.massFlowRate_kgps = 50.0 / 15.0;

  float timeScale = 1.0f;
  bool simulationRunning = false;
  double accumulatedTime = 0.0;

  // Launch mode
  enum class LaunchMode { DUMB, SMART };
  LaunchMode launchMode = LaunchMode::DUMB;

  // SmartArtillery configuration
  float targetX = 5000.0f;
  float targetY = 0.0f;
  float targetZ = 0.0f;
  GuidanceConfig guidanceConfig;
  guidanceConfig.proNavGain = 2.0; // Reduced from 3.0 for more stable guidance
  guidanceConfig.convergenceRadius_m = 10.0;
  ControlLimits controlLimits;
  controlLimits.maxFinDeflection_deg = 20.0;
  controlLimits.maxGLoad = 10.0;
  bool sensorNoiseEnabled = false;
  double calculatedElevation = 45.0;
  double calculatedRange = 0.0;
  bool isTargetReachable =
      true; // Tracks if smart artillery target is reachable

  // Main loop
  while (!glfwWindowShouldClose(window)) {
    glfwPollEvents();

    // Input processing
    if (!io.WantCaptureMouse) {
      // Handled in main loop body now
    }

    // Sim Step
    // Use fixed time step for physics stability recommended, but delta time is
    // fine for now We'll run simulation multiple times per frame if timescale
    // is high, or just use larger dt? Let's use 100Hz physics step
    constexpr double PHYSICS_DT = 0.01;

    if (simulationRunning) {
      // How much real time passed?
      // Simple approach: run X steps per frame based on timescale
      // Better: accumulator
      static double lastFrameTime = glfwGetTime();
      double currentFrameTime = glfwGetTime();
      double frameDelta = currentFrameTime - lastFrameTime;
      lastFrameTime = currentFrameTime;

      accumulatedTime += frameDelta * timeScale;

      while (accumulatedTime >= PHYSICS_DT) {
        world.step(PHYSICS_DT);
        accumulatedTime -= PHYSICS_DT;
      }
    } else {
      // Keep timer updated to avoid huge jump on resume
      // Fixed: Removed unused variable warning
      // static double lastFrameTime = glfwGetTime();
      // lastFrameTime = glfwGetTime();
    }

    // Start the Dear ImGui frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    // Dockspace
    ImGuiID dockSpaceId = ImGui::GetID("RocketSimDockSpace");
    ImGui::DockSpaceOverViewport(dockSpaceId, ImGui::GetMainViewport(),
                                 ImGuiDockNodeFlags_PassthruCentralNode);

    static bool isFirstRun = true;
    if (isFirstRun) {
      isFirstRun = false;
      ImGui::DockBuilderRemoveNode(dockSpaceId);
      // Fixed: Cast flags to avoid warning
      ImGui::DockBuilderAddNode(
          dockSpaceId, ImGuiDockNodeFlags_PassthruCentralNode |
                           (ImGuiDockNodeFlags)ImGuiDockNodeFlags_DockSpace);
      ImGui::DockBuilderSetNodeSize(dockSpaceId,
                                    ImGui::GetMainViewport()->Size);

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

    // Auto-Zoom Logic
    static bool autoZoom = true;
    double maxDist = 50.0; // Min distance

    // Update Camera (Manual Input)
    camera.processInput(window);

    // Calculate bounds if auto-zoom is on
    if (autoZoom) {
      bool hasData = false;
      for (const auto &p : world.getProjectiles()) {
        auto history = p->getHistory();
        if (!history.empty()) {
          hasData = true;
          for (const auto &s : history) {
            double d = s.position.norm();
            if (d > maxDist)
              maxDist = d;
          }
        }
      }

      if (hasData) {
        camera.setDistance((float)(maxDist * 1.5)); // 1.5x margin
      }
    }

    // Move Camera Update here so it reflects auto-zoom or manual input
    // (Note: Camera::processInput was redundant inside the loop if we update
    // here, but processInput handles mouse delta which is fine to call before
    // updating matrices)

    // Control Panel
    ImGui::Begin("Launch Controls");
    ImGui::Checkbox("Auto-Zoom 3D", &autoZoom);
    ImGui::Separator();

    // Preset configurations dropdown
    ImGui::Text("Preset Configurations");
    const char *presets[] = {"Custom", "Short-Range Mortar (2.5-8km)",
                             "Mid-Range Rocket (10-40km)"};
    static int currentPreset = 0;
    if (ImGui::Combo("##presets", &currentPreset, presets,
                     IM_ARRAYSIZE(presets))) {
      if (currentPreset == 1) {
        // Short-Range Mortar (2.5-8km)
        config.elevation_deg = 45.0;
        config.azimuth_deg = 0.0;
        config.dryMass_kg = 8.3;
        config.fuelMass_kg = 3.2;
        config.referenceArea_m2 = 0.01;
        config.thrust_N = 6200.0;
        config.burnDuration_s = 1.1;
        config.massFlowRate_kgps = 2.9;
      } else if (currentPreset == 2) {
        // Mid-Range Rocket (10-40km)
        config.elevation_deg = 45.0;
        config.azimuth_deg = 0.0;
        config.dryMass_kg = 45.5;
        config.fuelMass_kg = 20.5;
        config.referenceArea_m2 = 0.025;
        config.thrust_N = 24000.0;
        config.burnDuration_s = 2.0;
        config.massFlowRate_kgps = 10.5;
      }
    }
    ImGui::Separator();

    // Launch Mode Selection
    ImGui::Text("Launch Mode");
    if (ImGui::RadioButton("Dumb Artillery", launchMode == LaunchMode::DUMB)) {
      launchMode = LaunchMode::DUMB;
    }
    ImGui::SameLine();
    if (ImGui::RadioButton("Smart Artillery (Guided)",
                           launchMode == LaunchMode::SMART)) {
      launchMode = LaunchMode::SMART;
    }

    ImGui::Separator();

    if (launchMode == LaunchMode::DUMB) {
      // Manual angle control for dumb artillery
      ImGui::Text("Launch Direction");
      float el = (float)config.elevation_deg;
      if (ImGui::SliderFloat("Elevation (deg)", &el, 0.0f, 90.0f))
        config.elevation_deg = (double)el;

      float az = (float)config.azimuth_deg;
      if (ImGui::SliderFloat("Azimuth (deg)", &az, 0.0f, 360.0f))
        config.azimuth_deg = (double)az;
    } else {
      // Target coordinates for smart artillery
      ImGui::Text("Target Coordinates (ground position, meters)");

      // Track previous values to detect changes
      static float prevTargetX = targetX;
      static float prevTargetY = targetY;
      static LaunchConfig prevConfig = config;
      static double calculatedAzimuth = 0.0;

      ImGui::InputFloat("Target X (East)", &targetX, 100.0f, 1000.0f, "%.1f");
      ImGui::InputFloat("Target Y (North)", &targetY, 100.0f, 1000.0f, "%.1f");

      // Artillery always targets ground level
      targetZ = 0.0f;
      ImGui::TextDisabled("Target Z: 0.0 (ground level)");

      // Only recalculate if target or config changed (NOT every frame!)
      bool configChanged =
          (prevConfig.dryMass_kg != config.dryMass_kg ||
           prevConfig.fuelMass_kg != config.fuelMass_kg ||
           prevConfig.thrust_N != config.thrust_N ||
           prevConfig.burnDuration_s != config.burnDuration_s ||
           prevConfig.massFlowRate_kgps != config.massFlowRate_kgps ||
           prevConfig.referenceArea_m2 != config.referenceArea_m2);

      bool targetChanged = (prevTargetX != targetX || prevTargetY != targetY);

      if (targetChanged || configChanged) {
        // Calculate launch angles when target or config changes
        Eigen::Vector3d targetPos(targetX, targetY,
                                  0.0); // Z=0 for ground targets
        LaunchAngleSolver solver;

        // Check if target is reachable
        isTargetReachable = solver.isTargetReachable(config, targetPos);

        if (isTargetReachable) {
          // Get the angles (already calculated in isTargetReachable)
          calculatedElevation = config.elevation_deg;
          calculatedAzimuth = config.azimuth_deg;
          calculatedRange = solver.getCalculatedRange();
        }

        // Update previous values
        prevTargetX = targetX;
        prevTargetY = targetY;
        prevConfig = config;
      }

      // Display calculated values or error message
      ImGui::Separator();
      if (isTargetReachable) {
        ImGui::Text("Calculated Launch Parameters:");

        // For SmartArtillery, show reduced elevation (75% of ballistic optimal)
        double displayElevation = calculatedElevation * 0.75;
        ImGui::Text("  Elevation: %.2f° (reduced for guidance)",
                    displayElevation);
        ImGui::Text("  Azimuth: %.2f°", calculatedAzimuth);

        // Show actual horizontal range to target
        double actualRange = std::sqrt(targetX * targetX + targetY * targetY);
        ImGui::Text("  Target Range: %.1f m", actualRange);

        // Suppress warning
        (void)calculatedRange;
      } else {
        ImGui::TextColored(ImVec4(1.0f, 0.0f, 0.0f, 1.0f),
                           "TARGET UNREACHABLE!");
        ImGui::TextWrapped("The target is beyond the rocket's maximum range "
                           "with current configuration.");
        ImGui::TextWrapped("Try: Increase thrust, burn time, or reduce mass.");
      }

      ImGui::Separator();
      ImGui::Text("Guidance Parameters");
      float proNavGain = (float)guidanceConfig.proNavGain;
      if (ImGui::SliderFloat("ProNav Gain", &proNavGain, 1.0f, 6.0f, "%.1f"))
        guidanceConfig.proNavGain = (double)proNavGain;

      float convergenceRadius = (float)guidanceConfig.convergenceRadius_m;
      if (ImGui::SliderFloat("Convergence Radius (m)", &convergenceRadius, 1.0f,
                             50.0f, "%.1f"))
        guidanceConfig.convergenceRadius_m = (double)convergenceRadius;

      ImGui::Checkbox("Enable Sensor Noise", &sensorNoiseEnabled);
    }

    ImGui::Separator();
    // Sprint 3: Wind Configuration
    ImGui::Text("Environment");
    static float windSpeed = 0.0f;
    static float windDirection = 0.0f; // Azimuth in degrees

    if (ImGui::Button("Wind Configuration")) {
      ImGui::OpenPopup("Wind Config");
    }

    if (ImGui::BeginPopup("Wind Config")) {
      ImGui::Text("Wind Parameters");
      ImGui::Separator();
      bool changed = false;
      if (ImGui::SliderFloat("Wind Speed (m/s)", &windSpeed, 0.0f, 50.0f))
        changed = true;
      if (ImGui::SliderFloat("Wind Direction (deg)", &windDirection, 0.0f,
                             360.0f))
        changed = true;

      // Compass visual explanation
      ImGui::TextDisabled("(0=East, 90=North, 180=West, 270=South)");

      if (changed) {
        // Calculate wind vector (Z=0 for horizontal wind)
        double rad = (double)windDirection * M_PI / 180.0;
        Eigen::Vector3d w(windSpeed * std::cos(rad), windSpeed * std::sin(rad),
                          0.0);
        world.setWindVelocity(w);
      }
      ImGui::EndPopup();
    }
    // Show current wind summary inline
    if (std::abs(windSpeed) > 0.1f) {
      ImGui::TextColored(ImVec4(0.5f, 0.5f, 1.0f, 1.0f),
                         "Wind: %.1f m/s @ %.0f deg", windSpeed, windDirection);
    } else {
      ImGui::TextDisabled("Wind: Calm");
    }

    ImGui::Separator();
    ImGui::Text("Mass & Aerodynamics");
    float dryMass = (float)config.dryMass_kg;
    if (ImGui::SliderFloat("Dry Mass (kg)", &dryMass, 0.0f, 1000.0f, "%.2f"))
      config.dryMass_kg = (double)dryMass;

    float fuelMass = (float)config.fuelMass_kg;
    if (ImGui::SliderFloat("Fuel Mass (kg)", &fuelMass, 0.0f, 1000.0f, "%.2f"))
      config.fuelMass_kg = (double)fuelMass;

    float refArea = (float)config.referenceArea_m2;
    if (ImGui::SliderFloat("Ref Area (m^2)", &refArea, 0.0f, 0.5f, "%.4f"))
      config.referenceArea_m2 = (double)refArea;

    ImGui::Separator();
    ImGui::Text("Propulsion");
    float thrust = (float)config.thrust_N;
    if (ImGui::SliderFloat("Thrust (N)", &thrust, 0.0f, 50000.0f, "%.1f"))
      config.thrust_N = (double)thrust;

    float burnTime = (float)config.burnDuration_s;
    if (ImGui::SliderFloat("Burn Time (s)", &burnTime, 0.0f, 60.0f, "%.2f"))
      config.burnDuration_s = (double)burnTime;

    float massFlow = (float)config.massFlowRate_kgps;
    if (ImGui::SliderFloat("Mass Flow (kg/s)", &massFlow, 0.0f, 50.0f, "%.2f"))
      config.massFlowRate_kgps = (double)massFlow;

    if (ImGui::Button("FIRE")) {
      if (launchMode == LaunchMode::SMART) {
        // Check if target is reachable before launching
        if (!isTargetReachable) {
          // Show error - don't launch
          ImGui::OpenPopup("Unreachable Target");
        } else {
          // Recalculate angles right before launch with current config
          Eigen::Vector3d targetPos(targetX, targetY, targetZ);
          LaunchAngleSolver solver;
          auto [elevation, azimuth] =
              solver.calculateLaunchAngles(config, targetPos);

          config.elevation_deg = elevation;
          config.azimuth_deg = azimuth;

          // Set guidance target
          guidanceConfig.targetPosition = targetPos;

          // Create SmartArtillery with calculated angles and target
          auto projectile = std::make_unique<SmartArtillery>(
              config, guidanceConfig, controlLimits, sensorNoiseEnabled);
          world.addProjectile(std::move(projectile));
          simulationRunning = true;
        }
      } else {
        // Create DumbArtillery
        auto projectile = std::make_unique<DumbArtillery>(config);
        world.addProjectile(std::move(projectile));
        simulationRunning = true;
      }
    }

    // Popup for unreachable target
    if (ImGui::BeginPopup("Unreachable Target")) {
      ImGui::Text("Cannot launch: Target is UNREACHABLE!");
      ImGui::Separator();
      ImGui::TextWrapped("The target is beyond the rocket's range with current "
                         "configuration.");
      if (ImGui::Button("OK")) {
        ImGui::CloseCurrentPopup();
      }
      ImGui::EndPopup();
    }

    ImGui::SameLine();
    if (ImGui::Button("Reset World")) {
      world.clear();
      simulationRunning = false;
    }

    ImGui::Separator();
    ImGui::SliderFloat("Time Scale", &timeScale, 0.0f, 10.0f);
    ImGui::Text("Projectiles: %zu", world.getProjectiles().size());
    ImGui::Text("Sim Time: %.2f s",
                accumulatedTime); // This might be frame accumulator, not total
                                  // sim time. But we don't have total sim time
                                  // tracked easily in main loop variable. Let's
                                  // rely on history size * dt if exact info
                                  // needed or add totalTime variable.
    ImGui::End();

    // Live Telemetry
    ImGui::Begin("Live Telemetry");
    const auto &projs = world.getProjectiles();

    int activeCount = 0;
    for (const auto &p : projs) {
      if (!p->hasLanded())
        activeCount++;
    }

    if (activeCount == 0) {
      ImGui::Text("No active projectiles in flight.");
    } else {
      // Show data for the last launched projectile by default or loop all
      for (size_t i = 0; i < projs.size(); ++i) {
        const auto &p = projs[i];
        if (p->hasLanded())
          continue; // Filter: only show in-flight

        StateVector s = p->getState();
        double range = std::sqrt(s.position.x() * s.position.x() +
                                 s.position.y() * s.position.y());
        double speed = s.velocity.norm();
        double time = p->getHistory().size() * PHYSICS_DT;

        if (ImGui::CollapsingHeader(
                (std::string("Projectile ") + std::to_string(i)).c_str(),
                ImGuiTreeNodeFlags_DefaultOpen)) {
          ImGui::Text("Time:   %.2f s", time);
          ImGui::Separator();
          ImGui::Text("Position (m)");
          ImGui::Text("  X: %.2f", s.position.x());
          ImGui::Text("  Y: %.2f", s.position.y());
          ImGui::Text("  Z: %.2f (Altitude)", s.position.z());
          ImGui::Text("  Range: %.2f", range);
          ImGui::Separator();
          ImGui::Text("Velocity (m/s)");
          ImGui::Text("  X: %.2f", s.velocity.x());
          ImGui::Text("  Y: %.2f", s.velocity.y());
          ImGui::Text("  Z: %.2f", s.velocity.z());
          ImGui::Text("  Speed: %.2f", speed);
          ImGui::Separator();
          // Sprint 2: Additional telemetry
          ImGui::Text("Mach Number: %.2f",
                      dynamic_cast<DumbArtillery *>(p.get())->getMachNumber());
          ImGui::Text("Current Mass: %.2f kg", s.totalMass);
          ImGui::Text("Fuel Remaining: %.2f kg", s.fuelMass);
          auto *arty = dynamic_cast<DumbArtillery *>(p.get());
          if (arty) {
            ImGui::Text("Drag Force: %.2f N", arty->getDragForce().norm());
            ImGui::Text("Thrust: %.2f N", arty->getThrustForce().norm());
          }

          // SmartArtillery-specific telemetry
          auto *smartArty = dynamic_cast<SmartArtillery *>(p.get());
          if (smartArty) {
            ImGui::Separator();
            ImGui::Text("Smart Artillery Guidance");
            ImGui::Text("  Distance to Target: %.2f m",
                        smartArty->getDistanceToTarget());
            ImGui::Text("  G-Load: %.2f G", smartArty->getCurrentGLoad());
            auto finAngles = smartArty->getFinAngles();
            ImGui::Text("  Fin Pitch: %.2f°", finAngles.x() * 180.0 / M_PI);
            ImGui::Text("  Fin Yaw: %.2f°", finAngles.y() * 180.0 / M_PI);

            // Show guidance phase
            const char *guidanceStatus;
            if (!smartArty->isGuidanceEnabled()) {
              guidanceStatus = "DISABLED";
            } else if (smartArty->isGuidanceActive()) {
              guidanceStatus = "GUIDING";
            } else {
              guidanceStatus = "BOOST (waiting for velocity)";
            }
            ImGui::Text("  Guidance: %s", guidanceStatus);

            if (smartArty->hasReachedTarget()) {
              ImGui::TextColored(ImVec4(0.0f, 1.0f, 0.0f, 1.0f),
                                 "  STATUS: TARGET REACHED!");
            }
          }
        }
      }
    }
    ImGui::End();

    // 2D Analysis
    ImGui::Begin("Telemetry Analysis");

    // Throttle plot updates for better performance (update every 3 frames ~=
    // 20Hz)
    static int plotUpdateCounter = 0;
    static std::vector<std::vector<double>> cached_altitude_x,
        cached_altitude_y;
    static std::vector<std::vector<double>> cached_velocity_t,
        cached_velocity_v;
    static std::vector<std::vector<double>> cached_mach_alt, cached_mach_mach;
    static std::vector<std::vector<double>> cached_mass_t, cached_mass_m;

    plotUpdateCounter++;
    bool updatePlotData =
        (plotUpdateCounter % 3 == 0); // Update every 3rd frame

    if (updatePlotData) {
      const auto &projectiles = world.getProjectiles();

      // Clear cached data
      cached_altitude_x.clear();
      cached_altitude_y.clear();
      cached_velocity_t.clear();
      cached_velocity_v.clear();
      cached_mach_alt.clear();
      cached_mach_mach.clear();
      cached_mass_t.clear();
      cached_mass_m.clear();

      // Rebuild cache for all projectiles
      for (size_t i = 0; i < projectiles.size(); ++i) {
        auto history = projectiles[i]->getHistory();
        if (!history.empty()) {
          // Cache altitude vs range data
          std::vector<double> x, y;
          x.reserve(history.size());
          y.reserve(history.size());
          for (const auto &s : history) {
            double range = std::sqrt(s.position.x() * s.position.x() +
                                     s.position.y() * s.position.y());
            x.push_back(range);
            y.push_back(s.position.z());
          }
          cached_altitude_x.push_back(x);
          cached_altitude_y.push_back(y);

          // Cache velocity vs time data
          std::vector<double> t, v;
          t.reserve(history.size());
          v.reserve(history.size());
          for (size_t j = 0; j < history.size(); ++j) {
            t.push_back(j * PHYSICS_DT);
            v.push_back(history[j].velocity.norm());
          }
          cached_velocity_t.push_back(t);
          cached_velocity_v.push_back(v);

          // Cache mach number data
          auto *artillery = dynamic_cast<DumbArtillery *>(projectiles[i].get());
          if (artillery) {
            std::vector<double> alt, mach;
            alt.reserve(history.size());
            mach.reserve(history.size());
            for (const auto &s : history) {
              alt.push_back(s.position.z());
              double machNum = artillery->getAero().getMachNumber(
                  s.velocity, s.position.z());
              mach.push_back(machNum);
            }
            cached_mach_alt.push_back(alt);
            cached_mach_mach.push_back(mach);
          }

          // Cache mass vs time data
          std::vector<double> t_mass, m;
          t_mass.reserve(history.size());
          m.reserve(history.size());
          for (size_t j = 0; j < history.size(); ++j) {
            t_mass.push_back(j * PHYSICS_DT);
            m.push_back(history[j].totalMass);
          }
          cached_mass_t.push_back(t_mass);
          cached_mass_m.push_back(m);
        }
      }
    }

    if (ImPlot::BeginPlot("Altitude vs Range")) {
      ImPlot::SetupAxes("Ground Range (m)", "Altitude (m)",
                        ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);

      // Use cached data
      for (size_t i = 0; i < cached_altitude_x.size(); ++i) {
        char label[32];
        snprintf(label, 32, "P%zu", i);
        ImPlot::PlotLine(label, cached_altitude_x[i].data(),
                         cached_altitude_y[i].data(),
                         cached_altitude_x[i].size());
      }
      ImPlot::EndPlot();
    }

    if (ImPlot::BeginPlot("Velocity vs Time")) {
      ImPlot::SetupAxes("Time (s)", "Velocity (m/s)", ImPlotAxisFlags_AutoFit,
                        ImPlotAxisFlags_AutoFit);

      for (size_t i = 0; i < cached_velocity_t.size(); ++i) {
        char label[32];
        snprintf(label, 32, "P%zu", i);
        ImPlot::PlotLine(label, cached_velocity_t[i].data(),
                         cached_velocity_v[i].data(),
                         cached_velocity_t[i].size());
      }
      ImPlot::EndPlot();
    }

    // Sprint 2: Mach Number vs Altitude
    if (ImPlot::BeginPlot("Mach Number vs Altitude")) {
      ImPlot::SetupAxes("Altitude (m)", "Mach Number", ImPlotAxisFlags_AutoFit,
                        ImPlotAxisFlags_AutoFit);

      for (size_t i = 0; i < cached_mach_alt.size(); ++i) {
        char label[32];
        snprintf(label, 32, "P%zu", i);
        ImPlot::PlotLine(label, cached_mach_alt[i].data(),
                         cached_mach_mach[i].data(), cached_mach_alt[i].size());
      }
      ImPlot::EndPlot();
    }

    // Sprint 2: Mass vs Time
    if (ImPlot::BeginPlot("Mass vs Time")) {
      ImPlot::SetupAxes("Time (s)", "Mass (kg)", ImPlotAxisFlags_AutoFit,
                        ImPlotAxisFlags_AutoFit);

      for (size_t i = 0; i < cached_mass_t.size(); ++i) {
        char label[32];
        snprintf(label, 32, "P%zu", i);
        ImPlot::PlotLine(label, cached_mass_t[i].data(),
                         cached_mass_m[i].data(), cached_mass_t[i].size());
      }
      ImPlot::EndPlot();
    }

    ImGui::End();

    // Rendering
    ImGui::Render();
    int display_w, display_h;
    glfwGetFramebufferSize(window, &display_w, &display_h);
    glViewport(0, 0, display_w, display_h);
    glClearColor(0.05f, 0.05f, 0.08f,
                 1.0f); // Darker background for better contrast
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Update Camera
    camera.update();
    Eigen::Matrix4f view = camera.getViewMatrix();
    Eigen::Matrix4f proj =
        camera.getProjectionMatrix((float)display_w / (float)display_h);

    // Collect trajectories
    std::vector<std::vector<StateVector>> allTrajectories;
    for (const auto &p : world.getProjectiles()) {
      allTrajectories.push_back(p->getHistory());
    }

    renderer.render(view, proj, allTrajectories, camera.getDistance());

    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    glfwSwapBuffers(window);
  }

  // Cleanup
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImPlot::DestroyContext();
  ImGui::DestroyContext();

  glfwDestroyWindow(window);
  glfwTerminate();

  return 0;
}
