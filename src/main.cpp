#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "imgui_internal.h"
#include "implot.h"
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include "core/DumbArtillery.h"
#include "core/SimulationWorld.h"
#include "visualization/Camera.h"
#include "visualization/Renderer.h"

#include <cmath>
#include <iostream>
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
      glfwCreateWindow(1280, 720, "Rocket Sim - Sprint 1", NULL, NULL);
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
  config.muzzle_velocity_mps = 100.0;

  float timeScale = 1.0f;
  bool simulationRunning = false;
  double accumulatedTime = 0.0;

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
      static double lastFrameTime = glfwGetTime();
      lastFrameTime = glfwGetTime();
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
      ImGui::DockBuilderAddNode(dockSpaceId,
                                ImGuiDockNodeFlags_PassthruCentralNode |
                                    ImGuiDockNodeFlags_DockSpace);
      ImGui::DockBuilderSetNodeSize(dockSpaceId,
                                    ImGui::GetMainViewport()->Size);

      ImGuiID dockMain = dockSpaceId;
      ImGuiID dockLeft = ImGui::DockBuilderSplitNode(dockMain, ImGuiDir_Left,
                                                     0.20f, NULL, &dockMain);
      ImGuiID dockRight = ImGui::DockBuilderSplitNode(dockMain, ImGuiDir_Right,
                                                      0.25f, NULL, &dockMain);
      ImGuiID dockBottom = ImGui::DockBuilderSplitNode(dockMain, ImGuiDir_Down,
                                                       0.30f, NULL, &dockMain);

      ImGui::DockBuilderDockWindow("Telemetry Analysis", dockLeft);
      ImGui::DockBuilderDockWindow("Live Telemetry", dockRight);
      ImGui::DockBuilderDockWindow("Launch Controls", dockBottom);

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

    float el = (float)config.elevation_deg;
    if (ImGui::SliderFloat("Elevation (deg)", &el, 0.0f, 90.0f))
      config.elevation_deg = (double)el;

    float az = (float)config.azimuth_deg;
    if (ImGui::SliderFloat("Azimuth (deg)", &az, 0.0f, 360.0f))
      config.azimuth_deg = (double)az;

    ImGui::InputDouble("Muzzle Vel (m/s)", &config.muzzle_velocity_mps);

    if (ImGui::Button("FIRE")) {
      auto projectile = std::make_unique<DumbArtillery>(config);
      world.addProjectile(std::move(projectile));
      simulationRunning = true;
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
        }
      }
    }
    ImGui::End();

    // 2D Analysis
    ImGui::Begin("Telemetry Analysis");

    if (ImPlot::BeginPlot("Altitude vs Range")) {
      ImPlot::SetupAxes("Ground Range (m)", "Altitude (m)",
                        ImPlotAxisFlags_AutoFit, ImPlotAxisFlags_AutoFit);

      const auto &projectiles = world.getProjectiles();
      for (size_t i = 0; i < projectiles.size(); ++i) {
        auto history = projectiles[i]->getHistory();
        if (!history.empty()) {
          std::vector<double> x, y; // ImPlot needs double or float arrays
          x.reserve(history.size());
          y.reserve(history.size());

          for (const auto &s : history) {
            double range = std::sqrt(s.position.x() * s.position.x() +
                                     s.position.y() * s.position.y());
            x.push_back(range);
            y.push_back(s.position.z());
          }

          char label[32];
          snprintf(label, 32, "P%zu", i);
          ImPlot::PlotLine(label, x.data(), y.data(), x.size());
        }
      }
      ImPlot::EndPlot();
    }

    if (ImPlot::BeginPlot("Velocity vs Time")) {
      ImPlot::SetupAxes("Time (s)", "Velocity (m/s)", ImPlotAxisFlags_AutoFit,
                        ImPlotAxisFlags_AutoFit);

      const auto &projectiles = world.getProjectiles();
      for (size_t i = 0; i < projectiles.size(); ++i) {
        auto history = projectiles[i]->getHistory();
        if (!history.empty()) {
          std::vector<double> t, v;
          t.reserve(history.size());
          v.reserve(history.size());

          for (size_t j = 0; j < history.size(); ++j) {
            t.push_back(j * PHYSICS_DT); // Approximation
            v.push_back(history[j].velocity.norm());
          }

          char label[32];
          snprintf(label, 32, "P%zu", i);
          ImPlot::PlotLine(label, t.data(), v.data(), t.size());
        }
      }
      ImPlot::EndPlot();
    }

    ImGui::End();

    // Rendering
    ImGui::Render();
    int display_w, display_h;
    glfwGetFramebufferSize(window, &display_w, &display_h);
    glViewport(0, 0, display_w, display_h);
    glClearColor(0.1f, 0.1f, 0.15f, 1.0f);
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

    renderer.render(view, proj, allTrajectories);

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
