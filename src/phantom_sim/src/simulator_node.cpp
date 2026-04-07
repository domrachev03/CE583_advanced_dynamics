#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>
#include <thread>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "phantom_model/robot_model.hpp"

namespace phantom_sim {

class SimulatorNode : public rclcpp::Node {
public:
    SimulatorNode()
        : Node("phantom_simulator") {
        // --- Load model ---
        auto pkg = ament_index_cpp::get_package_share_directory("phantom_model");
        declare_parameter("config_file", pkg + "/config/phantom_params.yaml");
        std::string cfg = get_parameter("config_file").as_string();
        model_ = std::make_unique<phantom_model::RobotModel>(cfg);

        // --- Simulation parameters (ROS2 parameters, not model concern) ---
        declare_parameter("integration_dt", 0.0002);
        declare_parameter("publish_rate", 1000.0);
        declare_parameter("wait_for_input", false);
        declare_parameter("integrator", std::string("rk4"));
        dt_            = get_parameter("integration_dt").as_double();
        publish_rate_  = get_parameter("publish_rate").as_double();
        wait_for_input_ = get_parameter("wait_for_input").as_bool();

        auto integrator_str = get_parameter("integrator").as_string();
        if (integrator_str == "euler") {
            integrator_ = Integrator::EULER;
        } else if (integrator_str == "rk4") {
            integrator_ = Integrator::RK4;
        } else {
            RCLCPP_WARN(get_logger(),
                "Unknown integrator '%s', defaulting to rk4",
                integrator_str.c_str());
            integrator_ = Integrator::RK4;
        }

        // --- ROS I/O (handled by executor threads) ---
        torque_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            "phantom/torque", rclcpp::SensorDataQoS(),
            [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                std::lock_guard<std::mutex> lk(tau_mu_);
                for (size_t i = 0; i < 3 && i < msg->data.size(); ++i)
                    tau_(i) = msg->data[i];
                torque_received_.store(true, std::memory_order_release);
            });

        js_pub_    = create_publisher<sensor_msgs::msg::JointState>(
            "phantom/joint_states", rclcpp::SensorDataQoS());
        accel_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
            "phantom/joint_accel", rclcpp::SensorDataQoS());
        tf_bc_     = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // --- Publishing timer (runs on executor thread, not integration thread) ---
        auto pub_period = std::chrono::nanoseconds(
            static_cast<int64_t>(1e9 / publish_rate_));

        pub_timer_ = create_wall_timer(pub_period, [this]() {
            publish_state();
            publish_tf();
        });

        RCLCPP_INFO(get_logger(),
            "Robot model built (%d DOF) | dt=%.0f us | pub=%d Hz | integrator=%s",
            model_->ndof(), dt_ * 1e6,
            static_cast<int>(publish_rate_),
            integrator_ == Integrator::EULER ? "euler" : "rk4");

        // --- Start dedicated integration thread ---
        running_ = true;
        thread_  = std::thread(&SimulatorNode::integration_loop, this);

        // Stop integration thread as soon as rclcpp receives SIGTERM/SIGINT,
        // before the executor finishes unwinding.
        rclcpp::on_shutdown([this]() {
            running_.store(false, std::memory_order_release);
        });
    }

    ~SimulatorNode() override {
        running_ = false;
        if (thread_.joinable()) thread_.join();
    }

private:
    // ---- Semi-implicit (symplectic) Euler step --------------------------------
    // Update velocity first, then use the NEW velocity to advance position.
    // This preserves a modified energy, preventing numerical energy drift
    // that causes undamped systems to diverge.
    void euler_step(const Eigen::Vector3d& tau) {
        Eigen::Vector3d q, dq;
        {
            std::lock_guard<std::mutex> lk(state_mu_);
            q  = q_;
            dq = dq_;
        }

        Eigen::Vector3d ddq = model_->forward_dynamics(q, dq, tau);

        std::lock_guard<std::mutex> lk(state_mu_);
        ddq_ = ddq;             // snapshot for /phantom/joint_accel
        dq_  = dq + dt_ * ddq;  // velocity first (using old q)
        q_   = q  + dt_ * dq_;  // position with NEW velocity
    }

    // ---- RK4 step (called only from integration thread) --------------------
    void rk4_step(const Eigen::Vector3d& tau) {
        Eigen::Vector3d q, dq;
        {
            std::lock_guard<std::mutex> lk(state_mu_);
            q  = q_;
            dq = dq_;
        }

        auto eval = [&](const Eigen::Vector3d& qv, const Eigen::Vector3d& dqv) {
            return model_->forward_dynamics(qv, dqv, tau);
        };

        Eigen::Vector3d k1 = eval(q, dq);

        Eigen::Vector3d q2  = q  + (dt_ / 2) * dq;
        Eigen::Vector3d dq2 = dq + (dt_ / 2) * k1;
        Eigen::Vector3d k2  = eval(q2, dq2);

        Eigen::Vector3d q3  = q  + (dt_ / 2) * dq2;
        Eigen::Vector3d dq3 = dq + (dt_ / 2) * k2;
        Eigen::Vector3d k3  = eval(q3, dq3);

        Eigen::Vector3d q4  = q  + dt_ * dq3;
        Eigen::Vector3d dq4 = dq + dt_ * k3;
        Eigen::Vector3d k4  = eval(q4, dq4);

        std::lock_guard<std::mutex> lk(state_mu_);
        // k1 is the acceleration at the START of the integration step,
        // i.e. the ddq that corresponds to the (q, dq) about to be
        // overwritten.  Publishing it alongside the (q, dq) snapshot on
        // the same tick gives subscribers a self-consistent (q, dq, ddq)
        // triple.  Using the RK4-weighted average would drift from this
        // interpretation without any real accuracy benefit at dt=2e-4.
        ddq_ = k1;
        q_   = q  + (dt_ / 6) * (dq  + 2 * dq2 + 2 * dq3 + dq4);
        dq_  = dq + (dt_ / 6) * (k1  + 2 * k2  + 2 * k3  + k4);
    }

    // ---- Publishing (called by executor timer threads) ---------------------
    void publish_state() {
        Eigen::Vector3d q, dq, ddq, tau;
        {
            std::lock_guard<std::mutex> lk(state_mu_);
            q   = q_;
            dq  = dq_;
            ddq = ddq_;
        }
        {
            std::lock_guard<std::mutex> lk(tau_mu_);
            tau = tau_;
        }

        auto msg = sensor_msgs::msg::JointState();
        msg.header.stamp = now();
        msg.name     = {"joint1", "joint2", "joint3", "joint4", "joint5"};
        msg.position = {q(0), q(1), q(2),
                        M_PI / 2 - q(1) + q(2),
                        M_PI / 2 + q(1) - q(2)};
        msg.velocity = {dq(0), dq(1), dq(2),
                        -dq(1) + dq(2),
                         dq(1) - dq(2)};
        msg.effort   = {tau(0), tau(1), tau(2), 0.0, 0.0};
        js_pub_->publish(msg);

        // Acceleration of the 3 actuated DOF on a dedicated topic — kept
        // separate because sensor_msgs/JointState has no acceleration field.
        std_msgs::msg::Float64MultiArray accel_msg;
        accel_msg.data = {ddq(0), ddq(1), ddq(2)};
        accel_pub_->publish(accel_msg);
    }

    void publish_tf() {
        Eigen::Vector3d q;
        {
            std::lock_guard<std::mutex> lk(state_mu_);
            q = q_;
        }

        auto transforms = model_->forward_kinematics(q);

        static const std::pair<std::string, std::string> frames[] = {
            {"base_link", "link1"},
            {"link1",     "link2"},
            {"link1",     "link3"},
            {"link2",     "link4"},
            {"link3",     "link5"},
            {"link4",     "link6"},   // TCP (parallelogram closure)
        };

        auto stamp = now();
        for (size_t i = 0; i < transforms.size(); ++i) {
            geometry_msgs::msg::TransformStamped ts;
            ts.header.stamp    = stamp;
            ts.header.frame_id = frames[i].first;
            ts.child_frame_id  = frames[i].second;

            const Eigen::Matrix4d& T = transforms[i];
            const Eigen::Vector3d t = T.block<3, 1>(0, 3);
            const Eigen::Quaterniond quat(
                Eigen::Matrix3d(T.block<3, 3>(0, 0)));

            ts.transform.translation.x = t.x();
            ts.transform.translation.y = t.y();
            ts.transform.translation.z = t.z();
            ts.transform.rotation.x    = quat.x();
            ts.transform.rotation.y    = quat.y();
            ts.transform.rotation.z    = quat.z();
            ts.transform.rotation.w    = quat.w();

            tf_bc_->sendTransform(ts);
        }
    }

    // ---- Integration loop (dedicated thread, occupies one CPU core) --------
    void integration_loop() {
        if (wait_for_input_) {
            RCLCPP_INFO(get_logger(),
                "Waiting for torque command on phantom/torque ...");
            while (!torque_received_.load(std::memory_order_acquire)) {
                if (!running_.load(std::memory_order_relaxed)) return;
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
        RCLCPP_INFO(get_logger(), "Integration thread started");

        using clock = std::chrono::steady_clock;
        int64_t step = 0;
        auto next = clock::now();
        const auto step_dur =
            std::chrono::nanoseconds(static_cast<int64_t>(dt_ * 1e9));

        // Stats
        auto    stats_start    = clock::now();
        int64_t stats_steps    = 0;
        double  rk4_accum_us   = 0;
        constexpr double REPORT_S = 2.0;

        while (running_.load(std::memory_order_relaxed)) {
            // Read torque
            Eigen::Vector3d tau;
            {
                std::lock_guard<std::mutex> lk(tau_mu_);
                tau = tau_;
            }

            // Integration step (writes q_, dq_ under state_mu_)
            auto t0 = clock::now();
            if (integrator_ == Integrator::EULER) {
                euler_step(tau);
            } else {
                rk4_step(tau);
            }
            auto t1 = clock::now();
            rk4_accum_us += std::chrono::duration<double, std::micro>(t1 - t0).count();
            ++step;
            ++stats_steps;

            // Periodic stats (integration rate only; publishing has its own timers)
            auto now_t = clock::now();
            double elapsed = std::chrono::duration<double>(now_t - stats_start).count();
            if (elapsed >= REPORT_S) {
                double sim_hz   = stats_steps / elapsed;
                double rk4_avg  = rk4_accum_us / stats_steps;
                double rt_ratio = (stats_steps * dt_) / elapsed;

                Eigen::Vector3d q;
                {
                    std::lock_guard<std::mutex> lk(state_mu_);
                    q = q_;
                }

                RCLCPP_INFO(get_logger(),
                    "sim=%.0f Hz (%.2fx RT) | rk4=%.0f us/step | "
                    "q=[%.3f, %.3f, %.3f]",
                    sim_hz, rt_ratio, rk4_avg, q(0), q(1), q(2));

                stats_start  = now_t;
                stats_steps  = 0;
                rk4_accum_us = 0;
            }

            // Busy-wait for fixed-rate real-time execution
            next += step_dur;
            while (clock::now() < next) {
                if (!running_.load(std::memory_order_relaxed)) break;
            }
        }
    }

    // ---- Members -----------------------------------------------------------
    enum class Integrator { EULER, RK4 };

    std::unique_ptr<phantom_model::RobotModel> model_;
    double dt_{};
    double publish_rate_{};
    bool   wait_for_input_{false};
    Integrator integrator_{Integrator::RK4};

    // State: written by integration thread, read by timer callbacks
    Eigen::Vector3d q_   {Eigen::Vector3d::Zero()};
    Eigen::Vector3d dq_  {Eigen::Vector3d::Zero()};
    Eigen::Vector3d ddq_ {Eigen::Vector3d::Zero()};  // latest integration-step acceleration
    std::mutex state_mu_;

    // Torque input: written by subscription callback, read by integration thread
    Eigen::Vector3d tau_{Eigen::Vector3d::Zero()};
    std::mutex tau_mu_;

    // ROS I/O
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr torque_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr       js_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr   accel_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_bc_;
    rclcpp::TimerBase::SharedPtr pub_timer_;

    std::atomic<bool> running_{false};
    std::atomic<bool> torque_received_{false};
    std::thread thread_;
};

}  // namespace phantom_sim

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<phantom_sim::SimulatorNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
