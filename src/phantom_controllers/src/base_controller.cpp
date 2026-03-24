#include "phantom_controllers/base_controller.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace phantom_controllers {

BaseController::BaseController(const std::string& node_name, double rate_hz)
    : Node(node_name) {
    // Load robot model from phantom_model's shared config
    auto pkg = ament_index_cpp::get_package_share_directory("phantom_model");
    declare_parameter("config_file", pkg + "/config/phantom_params.yaml");
    std::string cfg = get_parameter("config_file").as_string();
    model_ = std::make_unique<phantom_model::RobotModel>(cfg);

    // Subscribe to joint states
    js_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "phantom/joint_states", rclcpp::SensorDataQoS(),
        std::bind(&BaseController::joint_state_callback, this,
                  std::placeholders::_1));

    // Torque publisher
    tau_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
        "phantom/torque", rclcpp::SensorDataQoS());

    // Control timer
    auto period = std::chrono::nanoseconds(
        static_cast<int64_t>(1e9 / rate_hz));
    timer_ = create_wall_timer(period,
        std::bind(&BaseController::control_tick, this));

    rate_hz_ = rate_hz;
}

void BaseController::joint_state_callback(
    const sensor_msgs::msg::JointState::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(state_mu_);
    for (size_t i = 0; i < 3 && i < msg->position.size(); ++i)
        q_[i] = msg->position[i];
    for (size_t i = 0; i < 3 && i < msg->velocity.size(); ++i)
        dq_[i] = msg->velocity[i];
    for (size_t i = 0; i < 3 && i < msg->effort.size(); ++i)
        effort_[i] = msg->effort[i];
    has_state_.store(true, std::memory_order_release);
}

void BaseController::control_tick() {
    if (!logged_ready_) {
        RCLCPP_INFO(get_logger(), "%s ready | rate=%d Hz | gravity_comp=%s",
                    get_name(), static_cast<int>(rate_hz_),
                    use_gravity_comp_ ? "on" : "off");
        logged_ready_ = true;
    }

    // Ask subclass for torque
    auto tau = compute_torque();

    // Optionally add gravity compensation
    if (use_gravity_comp_) {
        auto q = get_q();
        auto G = model_->gravity_vector({q[0], q[1], q[2]});
        for (int i = 0; i < 3; ++i)
            tau[i] += G[i];
    }

    // Publish
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {tau[0], tau[1], tau[2]};
    tau_pub_->publish(msg);
}

std::array<double, 3> BaseController::get_q() const {
    std::lock_guard<std::mutex> lk(state_mu_);
    return q_;
}

std::array<double, 3> BaseController::get_dq() const {
    std::lock_guard<std::mutex> lk(state_mu_);
    return dq_;
}

std::array<double, 3> BaseController::get_effort() const {
    std::lock_guard<std::mutex> lk(state_mu_);
    return effort_;
}

}  // namespace phantom_controllers
