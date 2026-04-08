#pragma once

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <random>
#include <stdexcept>
#include <string>

#include <Eigen/Dense>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "phantom_model/robot_model.hpp"

namespace phantom_controllers {

/// Base class for Phantom robot controllers.
///
/// Subscribes to joint states, publishes torque at a configurable rate,
/// and provides easy access to robot state for subclasses.
///
/// Instantiable as-is: by default `compute_torque()` returns zero, so a
/// vanilla `BaseController` acts as a no-op or pure gravity-compensation
/// node depending on the `gravity_compensation` ROS parameter (default
/// false). Subclasses override `compute_torque()` to add a control law.
///
/// Header-only: all methods are defined inline in this file.  Each
/// executable that includes this header gets its own compiled copy of
/// the implementation.
class BaseController : public rclcpp::Node {
public:
    /// @param node_name  ROS2 node name
    /// @param rate_hz    Control loop frequency (default 1000 Hz)
    explicit BaseController(const std::string& node_name,
                            double rate_hz = 1000.0);

    virtual ~BaseController() = default;

protected:
    // ---- Override this in your controller ----------------------------------

    /// Compute the control torque (3 DOF).
    /// Called at the configured rate. Default impl returns Vector3d::Zero();
    /// override in a subclass to add a control law.  Gravity compensation
    /// (if enabled) is added automatically after this.
    virtual Eigen::Vector3d compute_torque();

    // ---- State accessors (thread-safe snapshots) ---------------------------

    /// Current joint positions [rad] (3 actuated DOF)
    Eigen::Vector3d get_q() const;

    /// Current joint velocities [rad/s] (3 actuated DOF)
    Eigen::Vector3d get_dq() const;

    /// Current applied torque [N*m] (3 actuated DOF)
    Eigen::Vector3d get_effort() const;

    /// Whether at least one joint state has been received
    bool has_state() const { return has_state_.load(std::memory_order_acquire); }

    // ---- Gravity compensation toggle ---------------------------------------

    /// Enable or disable automatic gravity compensation.
    /// When enabled, G(q) is added to the torque returned by compute_torque().
    void enable_gravity_compensation(bool enable) { use_gravity_comp_ = enable; }
    bool gravity_compensation_enabled() const { return use_gravity_comp_; }

    // ---- Robot model access ------------------------------------------------

    std::unique_ptr<phantom_model::RobotModel> model_;

private:
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void control_tick();


    // Latest robot state (written by subscription, read by timer + subclass)
    Eigen::Vector3d q_      {Eigen::Vector3d::Zero()};
    Eigen::Vector3d dq_     {Eigen::Vector3d::Zero()};
    Eigen::Vector3d effort_ {Eigen::Vector3d::Zero()};
    mutable std::mutex state_mu_;
    std::atomic<bool> has_state_{false};

    bool use_gravity_comp_{false};
    double rate_hz_{1000.0};
    bool logged_ready_{false};

    // ROS I/O
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr tau_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

// ============================================================================
// Inline implementation
// ----------------------------------------------------------------------------
// Definitions live in the header so the package stays a single header-only
// unit — see the class docstring above.  Every method is `inline` so the
// linker is happy with multiple copies across translation units.
// ============================================================================

inline BaseController::BaseController(const std::string& node_name,
                                      double rate_hz)
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

    // Gravity compensation toggle (default off — opt in via launch param).
    declare_parameter("gravity_compensation", false);
    use_gravity_comp_ = get_parameter("gravity_compensation").as_bool();

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

inline void BaseController::joint_state_callback(
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

inline void BaseController::control_tick() {
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

inline Eigen::Vector3d BaseController::compute_torque() {
    return Eigen::Vector3d::Zero();
}

inline Eigen::Vector3d BaseController::get_q() const {
    std::lock_guard<std::mutex> lk(state_mu_);
    return q_;
}

inline Eigen::Vector3d BaseController::get_dq() const {
    std::lock_guard<std::mutex> lk(state_mu_);
    return dq_;
}

inline Eigen::Vector3d BaseController::get_effort() const {
    std::lock_guard<std::mutex> lk(state_mu_);
    return effort_;
}

}  // namespace phantom_controllers
