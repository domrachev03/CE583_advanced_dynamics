# Simulation Architecture

## Overview

The simulator integrates the Phantom robot's equations of motion in real-time using a 4th-order Runge-Kutta (RK4) method. It runs as a ROS2 node that accepts torque commands and publishes joint states and TF transforms.

## Threading Model

The simulator uses three threads:

```
┌──────────────────────────────┐
│  Integration Thread          │  Dedicated CPU core (busy-wait)
│  ┌────────────────────────┐  │
│  │ Read tau (mutex)       │  │
│  │ RK4 step (4x CasADi)  │  │  ~190 us per step
│  │ Write q, dq (mutex)    │  │
│  │ Busy-wait to fill dt   │  │
│  └────────────────────────┘  │
└──────────────────────────────┘

┌──────────────────────────────┐
│  Executor Thread(s)          │  Managed by MultiThreadedExecutor
│  ├─ Torque subscription      │  Writes tau_ (mutex-protected)
│  └─ Publish timer (1 kHz)    │  Reads q_, dq_ (mutex-protected)
│     ├─ JointState message    │
│     └─ TF transforms (5x)   │
└──────────────────────────────┘
```

- The **integration thread** runs a tight loop with busy-waiting to maintain a fixed timestep (default 0.2 ms = 5 kHz). It occupies one full CPU core.
- The **executor threads** handle ROS2 callbacks: torque subscription and publishing timers. Publishing runs at wall-clock rates independent of integration speed.
- State is shared via two mutexes: `state_mu_` (for q/dq) and `tau_mu_` (for torque input).

## RK4 Integration

Each step evaluates forward dynamics 4 times:

```
k1 = f(q, dq, tau)
k2 = f(q + dt/2 * k1_q, dq + dt/2 * k1_dq, tau)
k3 = f(q + dt/2 * k2_q, dq + dt/2 * k2_dq, tau)
k4 = f(q + dt * k3_q, dq + dt * k3_dq, tau)
```

where `f(q, dq, tau) = D^{-1}(q) * (tau - C(q,dq)*dq - G(q))` is the forward dynamics computed by the CasADi model (see [casadi_model.md](casadi_model.md)).

Torque is held constant during each RK4 step (zero-order hold from the latest received message).

## Topics

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `phantom/torque` | `Float64MultiArray` | Input | 3-element torque command [N*m] |
| `phantom/joint_states` | `JointState` | 1 kHz | 5 joints (3 actuated + 2 coupled via four-bar linkage) |
| `/tf` | `TransformStamped` | 1 kHz | 5 frames: base_link -> link1..link5 |

The JointState message includes the two coupled joints (joint4, joint5) whose positions are derived from the actuated joints:
- `joint4 = pi/2 - theta2 + theta3`
- `joint5 = pi/2 + theta2 - theta3`

## TF Tree

```
base_link
└── link1  (joint1: Rz)
    ├── link2  (joint2: Rx(-pi/2) * Rz)
    │   └── link4  (joint4: Rz, coupled)
    └── link3  (joint3: Rx(-pi/2) * Rz)
        └── link5  (joint5: Rz, coupled)
```

## Configuration

All parameters in `src/phantom_sim/config/phantom_params.yaml`:

```yaml
phantom:
  simulation:
    integration_dt: 0.0002   # RK4 timestep (seconds)
    publish_rate: 1000.0     # JointState + TF publishing (Hz)
    wait_for_input: false    # Wait for first torque before integrating
```

## Shutdown

On SIGTERM/SIGINT:
1. `rclcpp::on_shutdown` callback sets `running_ = false`
2. Integration thread exits busy-wait loop immediately (checks `running_` inside the spin)
3. Thread joins in destructor
4. Process exits in ~0.2 seconds

## Performance

On Apple M-series (arm64):

| Metric | Value |
|--------|-------|
| CasADi evaluation | ~48 us/call |
| RK4 step (4 calls) | ~190 us |
| Integration rate | 5,000 Hz |
| Real-time factor | 1.00x |
| Publishing overhead | ~6 us (JointState) + ~17 us (TF) |
