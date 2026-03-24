# Writing Custom Controllers

This guide explains how to write controllers for the Phantom robot simulation using the `BaseController` class from the `phantom_controllers` package.

## BaseController Overview

`BaseController` is a ROS2 node that handles all boilerplate:

- Subscribes to `phantom/joint_states` and keeps a thread-safe snapshot of `q`, `dq`, and `effort`
- Publishes torque commands to `phantom/torque` at a configurable rate (default 1 kHz)
- Loads the `RobotModel` from the shared config file (same model the simulator uses)
- Optionally adds gravity compensation `G(q)` on top of your control torque

You only need to implement one method: `compute_torque()`.

## Minimal Example

```cpp
#include "phantom_controllers/base_controller.hpp"

class MyController : public phantom_controllers::BaseController {
public:
    MyController()
        : BaseController("my_controller", 1000.0)  // name, rate in Hz
    {
        // Optional: enable gravity compensation
        // enable_gravity_compensation(true);
    }

protected:
    std::array<double, 3> compute_torque() override {
        // Read current state
        auto q  = get_q();    // joint positions [rad]
        auto dq = get_dq();   // joint velocities [rad/s]

        // Your control law here
        std::array<double, 3> tau = {0.0, 0.0, 0.0};

        // Example: PD control to hold q = 0
        for (int i = 0; i < 3; ++i)
            tau[i] = -10.0 * q[i] - 1.0 * dq[i];

        return tau;
        // If gravity comp is enabled, G(q) is added automatically
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MyController>());
    rclcpp::shutdown();
    return 0;
}
```

## Available State Accessors

All accessors return thread-safe snapshots (locked copies) of the latest data received from the simulator.

| Method | Returns | Description |
|--------|---------|-------------|
| `get_q()` | `std::array<double, 3>` | Joint positions (3 actuated DOF) in radians |
| `get_dq()` | `std::array<double, 3>` | Joint velocities in rad/s |
| `get_effort()` | `std::array<double, 3>` | Currently applied torques |
| `has_state()` | `bool` | Whether at least one JointState message has been received |
| `model()` | `const RobotModel&` | Access to the CasADi robot model |

## Gravity Compensation

Gravity compensation is **off by default**. Enable it in your constructor:

```cpp
enable_gravity_compensation(true);
```

When enabled, `G(q)` is computed from the current joint positions and **added** to whatever `compute_torque()` returns. This means:

- Return `{0, 0, 0}` from `compute_torque()` for pure gravity compensation (robot holds position)
- Return your control torque and gravity is handled on top -- you don't need to think about it

You can also toggle it at runtime:

```cpp
enable_gravity_compensation(false);  // disable mid-run
```

## Using the Robot Model

The base class exposes the full `RobotModel` for advanced use:

```cpp
std::array<double, 3> compute_torque() override {
    auto q = get_q();

    // Compute forward kinematics (5 transforms, column-major 4x4)
    auto fk = model().forward_kinematics({q[0], q[1], q[2]});

    // Compute gravity vector directly
    auto G = model().gravity_compensation({q[0], q[1], q[2]});

    // Compute forward dynamics
    auto ddq = model().forward_dynamics(
        {q[0], q[1], q[2]},    // q
        {0.0, 0.0, 0.0},       // dq
        {1.0, 0.0, 0.0});      // tau

    // ... use these in your control law
    return {0.0, 0.0, 0.0};
}
```

## Adding to CMakeLists.txt

In your `CMakeLists.txt`, link against `base_controller`:

```cmake
find_package(phantom_controllers REQUIRED)

add_executable(my_controller_node src/my_controller.cpp)
target_link_libraries(my_controller_node
    phantom_controllers::base_controller)
ament_target_dependencies(my_controller_node rclcpp)
```

Or if your controller lives inside the `phantom_controllers` package, just link the library directly:

```cmake
add_executable(my_controller_node src/my_controller.cpp)
target_link_libraries(my_controller_node base_controller)
ament_target_dependencies(my_controller_node rclcpp)
```

## Existing Controllers

| Controller | File | Description |
|-----------|------|-------------|
| `GravityCompController` | `gravity_comp_node.cpp` | Pure gravity compensation (returns zero torque with gravity comp enabled) |
