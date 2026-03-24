#pragma once

#include <array>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "phantom_sim/robot_model.hpp"

namespace phantom_controllers {

/// Base class for Phantom robot controllers.
///
/// Subscribes to joint states, publishes torque at a configurable rate,
/// and provides easy access to robot state for subclasses.
///
/// Subclasses must implement `compute_torque()` which returns the 3-DOF
/// torque command. Gravity compensation can be optionally added on top
/// by calling `enable_gravity_compensation(true)` in the subclass
/// constructor.
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
    /// Called at the configured rate. Return {0,0,0} for zero torque.
    /// Gravity compensation (if enabled) is added automatically after this.
    virtual std::array<double, 3> compute_torque() = 0;

    // ---- State accessors (thread-safe snapshots) ---------------------------

    /// Current joint positions [rad] (3 actuated DOF)
    std::array<double, 3> get_q() const;

    /// Current joint velocities [rad/s] (3 actuated DOF)
    std::array<double, 3> get_dq() const;

    /// Current applied torque [N*m] (3 actuated DOF)
    std::array<double, 3> get_effort() const;

    /// Whether at least one joint state has been received
    bool has_state() const { return has_state_.load(std::memory_order_acquire); }

    // ---- Gravity compensation toggle ---------------------------------------

    /// Enable or disable automatic gravity compensation.
    /// When enabled, G(q) is added to the torque returned by compute_torque().
    void enable_gravity_compensation(bool enable) { use_gravity_comp_ = enable; }
    bool gravity_compensation_enabled() const { return use_gravity_comp_; }

    // ---- Robot model access ------------------------------------------------

    /// Access the shared robot model (for computing dynamics, FK, etc.)
    const phantom_sim::RobotModel& model() const { return *model_; }

private:
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void control_tick();

    std::unique_ptr<phantom_sim::RobotModel> model_;

    // Latest robot state (written by subscription, read by timer + subclass)
    std::array<double, 3> q_      = {0, 0, 0};
    std::array<double, 3> dq_     = {0, 0, 0};
    std::array<double, 3> effort_ = {0, 0, 0};
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

}  // namespace phantom_controllers
