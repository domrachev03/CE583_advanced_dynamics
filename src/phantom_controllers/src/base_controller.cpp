#include "phantom_controllers/base_controller.hpp"

#include <random>
#include <stdexcept>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace phantom_controllers {

BaseController::BaseController(const std::string& node_name, double rate_hz)
    : Node(node_name) {
    // Load robot model from phantom_model's shared config
    auto pkg = ament_index_cpp::get_package_share_directory("phantom_model");
    declare_parameter("config_file", pkg + "/config/phantom_params.yaml");
    std::string cfg = get_parameter("config_file").as_string();

    // --- Parametric model-error injection ----------------------------------
    // Lets a controller build its RobotModel against perturbed parameters
    // while the simulator keeps ground truth.  Defaults (all mins = maxes
    // = 1.0) produce zero perturbation, so existing controller configs see
    // no behavioral change.  See phantom_model::ModelErrors for the
    // physical semantics.
    declare_parameter("model_error.scalar_min",   1.0);
    declare_parameter("model_error.scalar_max",   1.0);
    declare_parameter("model_error.per_link_min", 1.0);
    declare_parameter("model_error.per_link_max", 1.0);
    declare_parameter("model_error.seed",         static_cast<int64_t>(42));

    const double s_min = get_parameter("model_error.scalar_min").as_double();
    const double s_max = get_parameter("model_error.scalar_max").as_double();
    const double p_min = get_parameter("model_error.per_link_min").as_double();
    const double p_max = get_parameter("model_error.per_link_max").as_double();
    const int64_t seed = get_parameter("model_error.seed").as_int();

    if (s_min <= 0.0 || s_max < s_min) {
        throw std::invalid_argument(
            "model_error.scalar_{min,max}: both must be > 0 and min <= max");
    }
    if (p_min <= 0.0 || p_max < p_min) {
        throw std::invalid_argument(
            "model_error.per_link_{min,max}: both must be > 0 and min <= max");
    }

    std::mt19937_64 rng(static_cast<uint64_t>(seed));
    std::uniform_real_distribution<double> scalar_dist(s_min, s_max);
    std::uniform_real_distribution<double> per_link_dist(p_min, p_max);

    phantom_model::ModelErrors errors;
    errors.scalar_scale = scalar_dist(rng);
    for (int i = 0; i < 5; ++i) {
        errors.per_link_scale[i] = per_link_dist(rng);
    }

    RCLCPP_INFO(get_logger(),
        "model_error: k_global=%.4f, k_link=[%.4f %.4f %.4f %.4f %.4f], "
        "seed=%lld  (ranges: scalar=[%.3f,%.3f], per_link=[%.3f,%.3f])",
        errors.scalar_scale,
        errors.per_link_scale[0], errors.per_link_scale[1],
        errors.per_link_scale[2], errors.per_link_scale[3],
        errors.per_link_scale[4],
        static_cast<long long>(seed),
        s_min, s_max, p_min, p_max);

    model_ = std::make_unique<phantom_model::RobotModel>(cfg, errors);

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
        q_(i) = msg->position[i];
    for (size_t i = 0; i < 3 && i < msg->velocity.size(); ++i)
        dq_(i) = msg->velocity[i];
    for (size_t i = 0; i < 3 && i < msg->effort.size(); ++i)
        effort_(i) = msg->effort[i];
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
    Eigen::Vector3d tau = compute_torque();

    // Optionally add gravity compensation
    if (use_gravity_comp_) {
        tau += model_->gravity_vector(get_q());
    }

    // Publish
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {tau(0), tau(1), tau(2)};
    tau_pub_->publish(msg);
}

Eigen::Vector3d BaseController::get_q() const {
    std::lock_guard<std::mutex> lk(state_mu_);
    return q_;
}

Eigen::Vector3d BaseController::get_dq() const {
    std::lock_guard<std::mutex> lk(state_mu_);
    return dq_;
}

Eigen::Vector3d BaseController::get_effort() const {
    std::lock_guard<std::mutex> lk(state_mu_);
    return effort_;
}

}  // namespace phantom_controllers
