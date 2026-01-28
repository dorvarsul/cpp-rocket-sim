# Rocket Sim - Sprint 2

A C++ 3D ballistic simulation engine with atmospheric drag, variable mass dynamics, and real-time telemetry.

## Features

### Sprint 2: Atmospheric & Mass Dynamics
- **ISA Atmosphere Model**: International Standard Atmosphere with altitude-dependent density, temperature, and pressure.
- **Mach-Dependent Drag**: Realistic drag coefficients for subsonic, transonic, and supersonic flight.
- **Variable Mass**: Fuel consumption with real-time mass tracking and thrust-to-weight ratio dynamics.
- **Propulsion System**: Configurable thrust profiles with burn duration and mass flow rates.
- **Enhanced Telemetry**: Live Mach number, drag force, thrust, and remaining fuel displays.
- **Advanced Graphs**: Mach vs Altitude and Mass vs Time analysis plots.

### Sprint 1: Core Ballistics
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

Run the Sprint 1 unit tests:
```bash
./test_runner
```

Run the Sprint 2 verification tests:
```bash
./test_sprint2
```

## Launch Parameters

### Sprint 2 Parameters
- **Dry Mass** (kg): Rocket structure mass without fuel
- **Fuel Mass** (kg): Initial fuel load
- **Reference Area** (m²): Cross-sectional area for drag calculations
- **Thrust** (N): Rocket engine thrust force
- **Burn Time** (s): Duration of engine burn
- **Mass Flow** (kg/s): Fuel consumption rate

### Sprint 1 Parameters
- **Elevation** (degrees): Launch angle from horizontal (0-90°)
- **Azimuth** (degrees): Compass direction (0=East, 90=North)
- **Muzzle Velocity** (m/s): Initial velocity magnitude

## Controls

- **Orbit Camera**: Left-click and drag.
- **Zoom**: Scroll wheel (or +/- keys).
- **Launch**: Adjust parameters in the "Launch Controls" window and click **FIRE**.
- **Auto-Zoom**: Toggle "Auto-Zoom 3D" in the Launch Controls window.

## License

MIT License
