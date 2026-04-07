#include "phantom_controllers/base_controller.hpp"

namespace phantom_controllers {

/// Pure gravity compensation controller.
/// Returns zero control torque; gravity compensation is handled
/// automatically by the base class.
class GravityCompController : public BaseController {
public:
    GravityCompController()
        : BaseController("gravity_compensator", 1000.0) {
        enable_gravity_compensation(true);
    }

protected:
    Eigen::Vector3d compute_torque() override {
        return Eigen::Vector3d::Zero();
    }
};

}  // namespace phantom_controllers

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<phantom_controllers::GravityCompController>());
    rclcpp::shutdown();
    return 0;
}
