#include "phantom_model/robot_model.hpp"
#include <iostream>
#include <chrono>

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: test_model <config_path>\n";
        return 1;
    }

    std::cout << "Building model..." << std::flush;
    phantom_model::RobotModel model(argv[1]);
    std::cout << " done.\n";

    std::vector<double> q  = {0.0, 0.0, 0.0};
    std::vector<double> dq = {0.0, 0.0, 0.0};
    std::vector<double> tau = {0.0, 0.0, 0.0};

    std::cout << "Evaluating gravity compensation..." << std::flush;
    auto G = model.gravity_compensation(q);
    std::cout << " G = [" << G[0] << ", " << G[1] << ", " << G[2] << "]\n";

    std::cout << "Evaluating forward dynamics (zero torque)..." << std::flush;
    auto ddq = model.forward_dynamics(q, dq, tau);
    std::cout << " ddq = [" << ddq[0] << ", " << ddq[1] << ", " << ddq[2] << "]\n";

    std::cout << "Evaluating forward dynamics (gravity comp)..." << std::flush;
    auto ddq_gc = model.forward_dynamics(q, dq, G);
    std::cout << " ddq = [" << ddq_gc[0] << ", " << ddq_gc[1] << ", " << ddq_gc[2] << "]\n";

    std::cout << "Forward kinematics..." << std::flush;
    auto fk = model.forward_kinematics(q);
    for (int i = 0; i < 5; ++i) {
        std::cout << "  T_" << i << " pos(mm) = ["
                  << fk[i][12] << ", " << fk[i][13] << ", " << fk[i][14] << "]\n";
    }

    // Benchmark
    auto t0 = std::chrono::high_resolution_clock::now();
    int N = 10000;
    for (int i = 0; i < N; ++i)
        model.forward_dynamics(q, dq, tau);
    auto t1 = std::chrono::high_resolution_clock::now();
    double us = std::chrono::duration<double, std::micro>(t1 - t0).count() / N;
    std::cout << "forward_dynamics avg: " << us << " us/call\n";
    std::cout << "Max integration rate: " << 1e6 / (4 * us) << " Hz (4 calls/RK4 step)\n";

    return 0;
}
