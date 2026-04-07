// Vanilla controller executable: spins a `BaseController` directly.
//
// `BaseController` is instantiable on its own — its default `compute_torque()`
// returns zero.  Set the `gravity_compensation` ROS parameter to `true` (e.g.
// from a launch file or `--ros-args -p`) to make this node act as a pure
// gravity-compensation controller; leave it false for a no-op controller.
//
// Custom control laws live in subclasses of `BaseController` with their own
// executables — see docs/controllers.md.

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "phantom_controllers/base_controller.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<phantom_controllers::BaseController>(
        "phantom_controller"));
    rclcpp::shutdown();
    return 0;
}
