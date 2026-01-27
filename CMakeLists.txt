cmake_minimum_required(VERSION 3.20)
project(rocket_sim VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Warnings
add_compile_options(-Wall -Wextra -Wpedantic)

# Dependencies
include(FetchContent)

# Eigen
FetchContent_Declare(
  Eigen3
  URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
)
FetchContent_MakeAvailable(Eigen3)

# ImGui
FetchContent_Declare(
  imgui
  URL https://github.com/ocornut/imgui/archive/refs/heads/docking.zip
)
FetchContent_MakeAvailable(imgui)

# ImPlot
FetchContent_Declare(
  implot
  URL https://github.com/epezent/implot/archive/refs/heads/master.zip
)
FetchContent_MakeAvailable(implot)

# System Dependencies
find_package(glfw3 3.3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)

# Sources
file(GLOB_RECURSE SOURCES "src/*.cpp" "src/*.h")

add_executable(rocket_sim ${SOURCES})

# Include paths
target_include_directories(rocket_sim PRIVATE
  src
  ${imgui_SOURCE_DIR}
  ${imgui_SOURCE_DIR}/backends
  ${implot_SOURCE_DIR}
)

# Link libraries
target_link_libraries(rocket_sim PRIVATE
  Eigen3::Eigen
  glfw
  OpenGL::GL
  GLEW::GLEW
)

# ImGui Implementation Sources
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

# Definitions
target_compile_definitions(rocket_sim PRIVATE
  IMGUI_IMPL_OPENGL_LOADER_GLEW
)

# Test Runner
add_executable(test_runner tests/test_runner.cpp src/core/DumbArtillery.cpp src/core/SimulationWorld.cpp)
target_include_directories(test_runner PRIVATE src)
target_link_libraries(test_runner PRIVATE Eigen3::Eigen)
