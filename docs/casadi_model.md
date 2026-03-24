# CasADi Dynamics Model

## Overview

The `RobotModel` class builds a symbolic representation of the Phantom robot's dynamics using [CasADi](https://web.casadi.org/). The model is constructed once at startup and compiled into fast numerical functions that are evaluated thousands of times per second during simulation.

CasADi's `SX` type represents scalar symbolic expressions. When you build a matrix of `SX` elements and create a `Function`, CasADi optimizes the expression graph (common subexpression elimination) and produces an efficient evaluation routine.

## How the Model is Built

All computation happens in `RobotModel::build_model()`. The steps are:

### 1. Symbolic Variables

```cpp
SX q   = SX::sym("q", 3);    // joint positions
SX dq  = SX::sym("dq", 3);   // joint velocities
SX tau = SX::sym("tau", 3);   // applied torques
```

### 2. Kinematic Transforms

Five homogeneous transforms define the robot's kinematic chain. Helper functions `rot_z()`, `rot_x()`, and `make_transform()` build 4x4 matrices from rotation + translation:

| Transform | From -> To | Rotation | Translation |
|-----------|-----------|----------|-------------|
| T_01 | base -> link1 | Rz(theta1) | [0, 0, base_height] |
| T_12 | link1 -> link2 | Rx(-pi/2) * Rz(theta2 - pi/2) | [0, 0, upper_arm_offset] |
| T_13 | link1 -> link3 | Rx(-pi/2) * Rz(theta3) | [0, 0, upper_arm_offset] |
| T_24 | link2 -> link4 | Rz(pi/2 - theta2 + theta3) | [link4_x_offset, 0, 0] |
| T_35 | link3 -> link5 | Rz(pi/2 + theta2 - theta3) | [link5_x_offset, 0, 0] |

The T_24 and T_35 transforms encode the **four-bar linkage coupling** -- link4 and link5 positions depend on both theta2 and theta3.

### 3. COM Positions and Linear Velocity Jacobians

For each link, the world-frame COM position is computed by chaining transforms:

```cpp
SX T_01com = mtimes(T_01, T_com[0]);  // base -> link1 COM
SX Pc1 = T_01com(Slice(0,3), 3);      // extract position
```

The linear velocity Jacobian is computed via **CasADi automatic differentiation**:

```cpp
SX Jvc1 = jacobian(Pc1, q);  // 3x3 matrix: dPc1/dq
```

This replaces manual partial derivative computation with a single function call.

### 4. Angular Velocity Jacobians

For revolute joints, the angular velocity contribution is the joint's z-axis (in world frame) times the joint velocity. The **parallel linkage coupling** means:

| Link | Contributing joints | Reason |
|------|-------------------|--------|
| link1 | joint1 | Direct connection |
| link2 | joint1, joint2 | Serial chain |
| link3 | joint1, joint3 | Serial chain |
| link4 | joint1, **joint3** | Four-bar linkage |
| link5 | joint1, **joint2** | Four-bar linkage |

Note the cross-coupling: link4 uses joint3's axis, and link5 uses joint2's axis.

### 5. Inertia Matrix D(q)

The generalized inertia matrix combines translational and rotational kinetic energy:

```cpp
D = sum_i [ m_i * Jvc_i^T * Jvc_i + Jw_i^T * R_i * I_body_i * R_i^T * Jw_i ]
```

where `R_i * I_body_i * R_i^T` transforms the body-frame inertia tensor to the world frame.

### 6. Coriolis Matrix C(q, dq)

Computed via **Christoffel symbols of the first kind**:

```
C(j,k) = sum_i [ (dD(k,j)/dq(i) + dD(k,i)/dq(j) - dD(i,j)/dq(k)) / 2 ] * dq(i)
```

Each partial derivative `dD(k,j)/dq(i)` is computed using `casadi::jacobian()` -- automatic differentiation of the inertia matrix elements.

### 7. Gravity Vector G(q)

The gradient of potential energy:

```cpp
SX PE = sum_i [ m_i * dot(Pc_i, g) ];  // total potential energy
SX G  = gradient(PE, q);                // G = dPE/dq
```

`casadi::gradient()` handles the differentiation automatically.

### 8. Forward Dynamics

Finally, the acceleration is computed by solving the linear system:

```cpp
SX ddq = solve(D, tau - mtimes(C, dq) - G);
```

`solve(D, rhs)` is numerically more stable than `inv(D) * rhs` for the 3x3 system.

## Compiled CasADi Functions

Three `casadi::Function` objects are created from the symbolic expressions:

| Function | Inputs | Outputs | Purpose |
|----------|--------|---------|---------|
| `fwd_dyn_fn_` | q, dq, tau | ddq | Forward dynamics (used by RK4) |
| `gravity_fn_` | q | G | Gravity compensation torque |
| `fk_fn_` | q | 5 x vec16 | Forward kinematics (5 column-major 4x4 transforms) |

Each function evaluation takes ~48 us on Apple M-series.

## Parameters

All numerical values are loaded from `phantom_params.yaml`:

- **Masses**: 5 link masses in kg
- **COM positions**: center of mass in body frame, in meters
- **Inertia tensors**: 3x3 symmetric matrices in kg*m^2
- **Kinematic dimensions**: link offsets in meters
- **Gravity**: [0, 0, -9.81] m/s^2

## Modifying the Model

To change robot parameters, edit `src/phantom_sim/config/phantom_params.yaml`. The model is rebuilt at node startup.

To change the kinematic structure (e.g., add a link), modify `RobotModel::build_model()` in `robot_model.cpp`:
1. Add new transforms
2. Add COM/Jacobian computations
3. Include in D, C, G summations
4. Update the FK function outputs

CasADi handles all derivative computations automatically -- you only need to define the kinematics and link properties.
