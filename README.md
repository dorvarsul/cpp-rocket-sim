# Rocket Sim - Sprint 1

A C++ 3D ballistic simulation engine featuring RK4 integration, OpenGL visualization, and real-time telemetry.

## Features

- **Accurate Physics**: 4th-order Runge-Kutta numerical integration for stable trajectories.
- **3D Visualization**: Interactive orbital camera with trajectory ribbons and reference grid (OpenGL/GLFW).
- **Real-Time Telemetry**: Live dashboard showing flight status, position, and velocity for all projectiles.
- **Data Analysis**: Altitude vs Range and Velocity vs Time graphs (ImPlot).
- **Auto-Zoom**: 3D view automatically adjusts to keep all active projectiles in frame.

## Prerequisites

- **CMake** (3.12+)
- **C++ Compiler** (supporting C++17 or later)
- **OpenGL Libraries**:
  - `libgl1-mesa-dev`
  - `libglu1-mesa-dev`
  - `libx11-dev`
  - `libxrandr-dev`
  - `libxinerama-dev`
  - `libxcursor-dev`
  - `libxi-dev`

## Building

```bash
mkdir build
cd build
cmake ..
make -j4
```

## Running

Run the simulation:
```bash
./rocket_sim
```

Run the unit tests:
```bash
./test_runner
```

## Controls

- **Orbit Camera**: Left-click and drag.
- **Zoom**: Scroll wheel (or +/- keys).
- **Launch**: Adjust parameters in the "Launch Controls" window and click **FIRE**.
- **Auto-Zoom**: Toggle "Auto-Zoom 3D" in the Launch Controls window.

## License

MIT License
