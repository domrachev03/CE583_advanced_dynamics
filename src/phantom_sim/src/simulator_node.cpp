#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "phantom_sim/robot_model.hpp"

namespace phantom_sim {

// Column-major 4×4 → quaternion  (Shepperd's method)
static geometry_msgs::msg::Quaternion mat_to_quat(const double* T) {
    double r00 = T[0], r10 = T[1], r20 = T[2];
    double r01 = T[4], r11 = T[5], r21 = T[6];
    double r02 = T[8], r12 = T[9], r22 = T[10];

    geometry_msgs::msg::Quaternion quat;
    double tr = r00 + r11 + r22;
    if (tr > 0) {
        double s = 0.5 / std::sqrt(tr + 1.0);
        quat.w = 0.25 / s;
        quat.x = (r21 - r12) * s;
        quat.y = (r02 - r20) * s;
        quat.z = (r10 - r01) * s;
    } else if (r00 > r11 && r00 > r22) {
        double s = 2.0 * std::sqrt(1.0 + r00 - r11 - r22);
        quat.w = (r21 - r12) / s;
        quat.x = 0.25 * s;
        quat.y = (r01 + r10) / s;
        quat.z = (r02 + r20) / s;
    } else if (r11 > r22) {
        double s = 2.0 * std::sqrt(1.0 + r11 - r00 - r22);
        quat.w = (r02 - r20) / s;
        quat.x = (r01 + r10) / s;
        quat.y = 0.25 * s;
        quat.z = (r12 + r21) / s;
    } else {
        double s = 2.0 * std::sqrt(1.0 + r22 - r00 - r11);
        quat.w = (r10 - r01) / s;
        quat.x = (r02 + r20) / s;
        quat.y = (r12 + r21) / s;
        quat.z = 0.25 * s;
    }
    return quat;
}

class SimulatorNode : public rclcpp::Node {
public:
    SimulatorNode()
        : Node("phantom_simulator") {
        // --- Load model ---
        auto pkg = ament_index_cpp::get_package_share_directory("phantom_sim");
        declare_parameter("config_file", pkg + "/config/phantom_params.yaml");
        std::string cfg = get_parameter("config_file").as_string();
        model_ = std::make_unique<RobotModel>(cfg);
        RCLCPP_INFO(get_logger(), "Robot model built (%d DOF)", model_->ndof());

        dt_ = model_->sim_params().integration_dt;
        pub_skip_  = std::max(1, static_cast<int>(
            1.0 / (dt_ * model_->sim_params().publish_rate)));
        tf_skip_   = std::max(1, static_cast<int>(
            1.0 / (dt_ * model_->sim_params().tf_rate)));

        // --- ROS I/O ---
        torque_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            "phantom/torque", rclcpp::SensorDataQoS(),
            [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
                std::lock_guard<std::mutex> lk(tau_mu_);
                for (size_t i = 0; i < 3 && i < msg->data.size(); ++i)
                    tau_[i] = msg->data[i];
                torque_received_.store(true, std::memory_order_release);
            });

        js_pub_ = create_publisher<sensor_msgs::msg::JointState>(
            "phantom/joint_states", rclcpp::SensorDataQoS());
        tf_bc_  = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // --- Start integration thread ---
        running_ = true;
        thread_  = std::thread(&SimulatorNode::integration_loop, this);
    }

    ~SimulatorNode() override {
        running_ = false;
        if (thread_.joinable()) thread_.join();
    }

private:
    // ---- RK4 step ----------------------------------------------------------
    void rk4_step(const std::array<double, 3>& tau) {
        auto eval = [&](const std::vector<double>& qv,
                        const std::vector<double>& dqv) {
            return model_->forward_dynamics(qv, dqv,
                       {tau[0], tau[1], tau[2]});
        };

        std::vector<double> qv(q_.begin(), q_.end());
        std::vector<double> dqv(dq_.begin(), dq_.end());

        // k1
        auto ddq1 = eval(qv, dqv);

        // k2
        std::vector<double> q2(3), dq2(3);
        for (int i = 0; i < 3; ++i) {
            q2[i]  = qv[i]  + dt_ / 2 * dqv[i];
            dq2[i] = dqv[i] + dt_ / 2 * ddq1[i];
        }
        auto ddq2 = eval(q2, dq2);

        // k3
        std::vector<double> q3(3), dq3(3);
        for (int i = 0; i < 3; ++i) {
            q3[i]  = qv[i]  + dt_ / 2 * dq2[i];
            dq3[i] = dqv[i] + dt_ / 2 * ddq2[i];
        }
        auto ddq3 = eval(q3, dq3);

        // k4
        std::vector<double> q4(3), dq4(3);
        for (int i = 0; i < 3; ++i) {
            q4[i]  = qv[i]  + dt_ * dq3[i];
            dq4[i] = dqv[i] + dt_ * ddq3[i];
        }
        auto ddq4 = eval(q4, dq4);

        // update
        for (int i = 0; i < 3; ++i) {
            q_[i]  = qv[i]  + dt_ / 6 * (dqv[i]  + 2*dq2[i]  + 2*dq3[i]  + dq4[i]);
            dq_[i] = dqv[i] + dt_ / 6 * (ddq1[i] + 2*ddq2[i] + 2*ddq3[i] + ddq4[i]);
        }
    }

    // ---- Publishing --------------------------------------------------------
    void publish_state() {
        static int pub_count = 0;
        if (++pub_count == 1 || pub_count % 1000 == 0) {
            RCLCPP_INFO(get_logger(), "step %d  q=[%.4f, %.4f, %.4f]  dq=[%.6f, %.6f, %.6f]",
                        pub_count, q_[0], q_[1], q_[2], dq_[0], dq_[1], dq_[2]);
        }
        auto msg = sensor_msgs::msg::JointState();
        msg.header.stamp = now();

        // 3 actuated + 2 coupled (four-bar linkage)
        msg.name     = {"joint1", "joint2", "joint3", "joint4", "joint5"};
        msg.position = {q_[0], q_[1], q_[2],
                        M_PI / 2 - q_[1] + q_[2],
                        M_PI / 2 + q_[1] - q_[2]};
        msg.velocity = {dq_[0], dq_[1], dq_[2],
                        -dq_[1] + dq_[2],
                         dq_[1] - dq_[2]};
        {
            std::lock_guard<std::mutex> lk(tau_mu_);
            msg.effort = {tau_[0], tau_[1], tau_[2], 0.0, 0.0};
        }
        js_pub_->publish(msg);
    }

    void publish_tf() {
        auto transforms = model_->forward_kinematics(
            {q_[0], q_[1], q_[2]});

        // parent → child mapping  (matches FK function output order)
        static const std::pair<std::string, std::string> frames[] = {
            {"base_link", "link1"},
            {"link1",     "link2"},
            {"link1",     "link3"},
            {"link2",     "link4"},
            {"link3",     "link5"},
        };

        auto stamp = now();
        for (int i = 0; i < 5; ++i) {
            geometry_msgs::msg::TransformStamped ts;
            ts.header.stamp    = stamp;
            ts.header.frame_id = frames[i].first;
            ts.child_frame_id  = frames[i].second;

            const auto& T = transforms[i];
            // positions: mm → m
            ts.transform.translation.x = T[12] * 0.001;
            ts.transform.translation.y = T[13] * 0.001;
            ts.transform.translation.z = T[14] * 0.001;
            ts.transform.rotation      = mat_to_quat(T.data());

            tf_bc_->sendTransform(ts);
        }
    }

    // ---- Main loop (occupies one CPU core) ---------------------------------
    void integration_loop() {
        // Wait for first torque command so controller can start first
        RCLCPP_INFO(get_logger(), "Waiting for torque command on phantom/torque ...");
        while (!torque_received_.load(std::memory_order_acquire)) {
            if (!running_.load(std::memory_order_relaxed)) return;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        RCLCPP_INFO(get_logger(), "Torque received – starting integration (dt=%.1f us)",
                     dt_ * 1e6);

        int step = 0;
        auto next = std::chrono::steady_clock::now();
        const auto step_dur =
            std::chrono::nanoseconds(static_cast<int64_t>(dt_ * 1e9));

        while (running_.load(std::memory_order_relaxed)) {
            // read latest torque
            std::array<double, 3> tau{};
            {
                std::lock_guard<std::mutex> lk(tau_mu_);
                tau = tau_;
            }

            rk4_step(tau);
            ++step;

            if (step % pub_skip_ == 0)  publish_state();
            if (step % tf_skip_  == 0)  publish_tf();

            // busy-wait for fixed-rate real-time execution
            next += step_dur;
            while (std::chrono::steady_clock::now() < next) { /* spin */ }
        }
    }

    // ---- Members -----------------------------------------------------------
    std::unique_ptr<RobotModel> model_;
    double dt_{};
    int pub_skip_{1};
    int tf_skip_{1};

    std::array<double, 3> q_  = {0, 0, 0};
    std::array<double, 3> dq_ = {0, 0, 0};

    std::array<double, 3> tau_ = {0, 0, 0};
    std::mutex tau_mu_;

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr torque_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_bc_;

    std::atomic<bool> running_{false};
    std::atomic<bool> torque_received_{false};
    std::thread thread_;
};

}  // namespace phantom_sim

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<phantom_sim::SimulatorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
