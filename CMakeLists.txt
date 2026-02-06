cmake_minimum_required(VERSION 3.14)
project(RocketSim)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- Dependencies ---
include(FetchContent)

# ImGui
FetchContent_Declare(
  imgui
  GIT_REPOSITORY https://github.com/ocornut/imgui
  GIT_TAG        docking
)
FetchContent_MakeAvailable(imgui)

# ImPlot
FetchContent_Declare(
  implot
  GIT_REPOSITORY https://github.com/epezent/implot
  GIT_TAG        master
)
FetchContent_MakeAvailable(implot)

# System Libraries
find_package(Eigen3 REQUIRED)
find_package(glfw3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)

# --- Sources ---
file(GLOB_RECURSE SOURCES
  "src/main.cpp"
  "src/guidance/*.cpp" "src/guidance/*.h"
  "src/physics/*.cpp" "src/physics/*.h"
  "src/rendering/*.cpp" "src/rendering/*.h"
  "src/simulation/*.cpp" "src/simulation/*.h"
  "src/utils/*.cpp" "src/utils/*.h"
)

# --- Main Executable ---
add_executable(rocket_sim ${SOURCES})

target_include_directories(rocket_sim PRIVATE
  src/guidance
  src/physics
  src/rendering
  src/simulation
  src/utils
  ${imgui_SOURCE_DIR}
  ${imgui_SOURCE_DIR}/backends
  ${implot_SOURCE_DIR}
  ${EIGEN3_INCLUDE_DIR} 
)

target_link_libraries(rocket_sim PRIVATE
  Eigen3::Eigen
  glfw
  OpenGL::GL
  GLEW::GLEW
)

# ImGui/ImPlot Sources
target_sources(rocket_sim PRIVATE
  ${imgui_SOURCE_DIR}/imgui.cpp
  ${imgui_SOURCE_DIR}/imgui_demo.cpp
  ${imgui_SOURCE_DIR}/imgui_draw.cpp
  ${imgui_SOURCE_DIR}/imgui_tables.cpp
  ${imgui_SOURCE_DIR}/imgui_widgets.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
  ${implot_SOURCE_DIR}/implot.cpp
  ${implot_SOURCE_DIR}/implot_items.cpp
  ${implot_SOURCE_DIR}/implot_demo.cpp
)

target_compile_definitions(rocket_sim PRIVATE
  IMGUI_IMPL_OPENGL_LOADER_GLEW
)

# --- Tests ---
# Helper function to define tests to reduce boilerplate
function(add_rocket_test executable_name main_file)
    add_executable(${executable_name} 
      ${main_file}
      src/simulation/DumbArtillery.cpp 
      src/simulation/SmartArtillery.cpp
      src/simulation/SimulationWorld.cpp
      src/physics/AtmosphereModel.cpp
      src/physics/AeroComponent.cpp
      src/physics/PropulsionSystem.cpp
      src/physics/MassProperties.cpp
      src/guidance/GuidanceUnit.cpp
      src/guidance/NavigationSystem.cpp
      src/guidance/ControlSystem.cpp
      src/utils/LaunchAngleSolver.cpp
      ${ARGN} # Extra sources
    )
    target_include_directories(${executable_name} PRIVATE
      src/guidance
      src/physics
      src/rendering
      src/simulation
      src/utils
      ${EIGEN3_INCLUDE_DIR}
    )
    target_link_libraries(${executable_name} PRIVATE Eigen3::Eigen)
endfunction()

# Test Runner
add_rocket_test(test_runner tests/test_runner.cpp)

# Sprint 2 Test Runner
add_rocket_test(test_sprint2 tests/test_sprint2.cpp)

# Verify Smart
add_rocket_test(verify_smart tests/verify_smart_targeting.cpp)

# Smart Guidance Verification
# Note: This one had a slightly different list of source files in the original file, 
# but they seem to be a subset or reordered. I'll maintain the original explicit list 
# for correctness if the helper doesn't match perfectly, or just use the helper if it overlaps enough.
# Looking at the original, it uses `utils/LaunchAngleSolver.cpp` and physics/guidance modules. 
# The helper includes these. It also includes DUmby/SmartArtillery/SimulationWorld.
# The original test_smart_guidance INCLUDED SimulationWorld, Dumb, Smart. 
# So the helper matches!
add_rocket_test(test_smart_guidance tests/test_smart_guidance.cpp)

# Debug Solver
add_rocket_test(debug_solver tests/debug_solver.cpp)
