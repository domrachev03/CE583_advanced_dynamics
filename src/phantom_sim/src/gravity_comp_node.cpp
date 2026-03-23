#include <array>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "phantom_sim/robot_model.hpp"

namespace phantom_sim {

class GravityCompNode : public rclcpp::Node {
public:
    GravityCompNode()
        : Node("gravity_compensator") {
        // --- Load the same model as the simulator ---
        auto pkg = ament_index_cpp::get_package_share_directory("phantom_sim");
        declare_parameter("config_file", pkg + "/config/phantom_params.yaml");
        std::string cfg = get_parameter("config_file").as_string();
        model_ = std::make_unique<RobotModel>(cfg);
        RCLCPP_INFO(get_logger(), "Gravity-comp model built (%d DOF)",
                     model_->ndof());

        // --- ROS I/O ---
        js_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "phantom/joint_states", rclcpp::SensorDataQoS(),
            [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
                std::lock_guard<std::mutex> lk(q_mu_);
                for (size_t i = 0; i < 3 && i < msg->position.size(); ++i)
                    q_[i] = msg->position[i];
                // q_ updated with latest joint state
            });

        tau_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
            "phantom/torque", rclcpp::SensorDataQoS());

        // 1 kHz control timer
        timer_ = create_wall_timer(
            std::chrono::microseconds(1000),
            std::bind(&GravityCompNode::control_tick, this));
    }

private:
    void control_tick() {
        // Publish G(q) even before receiving joint states (uses q=0 initially),
        // so the simulator can start integration with gravity already compensated.
        std::array<double, 3> q{};
        {
            std::lock_guard<std::mutex> lk(q_mu_);
            q = q_;
        }

        auto G = model_->gravity_compensation({q[0], q[1], q[2]});

        std_msgs::msg::Float64MultiArray msg;
        msg.data = std::move(G);
        tau_pub_->publish(msg);
    }

    std::unique_ptr<RobotModel> model_;

    std::array<double, 3> q_ = {0, 0, 0};
    std::mutex q_mu_;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr tau_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace phantom_sim

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<phantom_sim::GravityCompNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
