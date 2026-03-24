# Phantom Robot Simulation

CasADi-based numerical simulation of a 5-link Phantom haptic device, running as ROS2 nodes with RK4 integration.

## Architecture

The Phantom robot has 3 actuated joints and 5 links connected through a parallel four-bar linkage mechanism. The dynamics are derived from the Euler-Lagrange formulation:

```
D(q) * ddq + C(q, dq) * dq + G(q) = tau
```

where D, C, G are built symbolically using CasADi with automatic differentiation for Jacobians and Christoffel symbols. All physical parameters (masses, inertias, COM positions, link dimensions) are loaded from a shared YAML config file.

### ROS2 Nodes

| Node | Description | Rate |
|------|-------------|------|
| `simulator_node` | RK4 integration of robot dynamics on a dedicated CPU core | 5 kHz integration, 1 kHz publishing |
| `gravity_comp_node` | Publishes gravity compensation torque `tau = G(q)` | 1 kHz |

### Topics

| Topic | Type | Direction |
|-------|------|-----------|
| `phantom/torque` | `std_msgs/Float64MultiArray` | Input to simulator (3 joint torques) |
| `phantom/joint_states` | `sensor_msgs/JointState` | Output (5 joints: 3 actuated + 2 coupled) |
| `/tf` | TF2 transforms | Output (5 frames: base_link -> link1-5) |

### Config

All parameters are in `src/phantom_sim/config/phantom_params.yaml` (SI units: meters, kg, kg*m^2). Both the simulator and controller load this same file.

Key simulation settings:
- `integration_dt` - RK4 timestep (default 0.2 ms = 5 kHz)
- `publish_rate` - JointState + TF publishing rate (default 1 kHz)
- `wait_for_input` - if `true`, simulator waits for first torque message before integrating; if `false`, starts immediately (robot falls under gravity)

## Prerequisites

- [pixi](https://pixi.sh) package manager
- macOS (arm64) — other platforms may work with `platforms` adjustment in `pixi.toml`

## Quick Start

```bash
# Install all dependencies (ROS2 Humble, CasADi, build tools)
pixi install

# Build
pixi run build

# Run simulator only (robot falls under gravity)
pixi run sim

# Run everything (simulator + gravity comp + RVIZ2)
pixi run all
```

### Running Individual Components

Each command needs its own terminal:

```bash
# Terminal 1: Simulator
pixi run sim

# Terminal 2: Gravity compensation controller
pixi run gravity_comp

# Terminal 3: RVIZ2 visualization
pixi run rviz
```

### Standalone Model Test

Evaluates the CasADi model outside of ROS2 and benchmarks forward dynamics:

```bash
pixi exec bash -c './build/phantom_sim/test_model src/phantom_sim/config/phantom_params.yaml'
```

## Project Structure

```
phantom_sim/
├── pixi.toml                          # Dependencies and tasks
├── cyclonedds.xml                     # Localhost DDS config (macOS)
├── scripts/activate.sh                # Pixi env activation
├── reference_code/inertia.m           # Original MATLAB reference
└── src/phantom_sim/
    ├── CMakeLists.txt
    ├── package.xml
    ├── config/phantom_params.yaml     # Robot parameters (shared)
    ├── include/phantom_sim/
    │   └── robot_model.hpp            # CasADi model API
    ├── src/
    │   ├── robot_model.cpp            # Symbolic dynamics (D, C, G)
    │   ├── simulator_node.cpp         # RK4 integration + publishing
    │   ├── gravity_comp_node.cpp      # Gravity compensation controller
    │   └── test_model.cpp             # Standalone model test
    ├── launch/sim.launch.py           # Launch all nodes + RVIZ2
    └── rviz/phantom.rviz              # TF tree display config
```

## Performance

Measured on Apple M-series (arm64):

| Metric | Value |
|--------|-------|
| CasADi `forward_dynamics` call | ~48 us |
| RK4 step (4 calls) | ~190 us |
| Integration rate | 5,000 Hz (1.0x real-time) |
| JointState publish rate | 1,000 Hz |
| TF publish rate | 1,000 Hz |
| Shutdown time (SIGTERM) | ~0.2 s |
