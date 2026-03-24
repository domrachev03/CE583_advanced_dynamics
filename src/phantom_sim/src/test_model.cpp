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

    std::cout << "Inertia matrix D(q)..." << std::flush;
    auto D = model.inertia_matrix(q);
    std::cout << "\n  [" << D[0] << ", " << D[1] << ", " << D[2] << "]\n"
              << "  [" << D[3] << ", " << D[4] << ", " << D[5] << "]\n"
              << "  [" << D[6] << ", " << D[7] << ", " << D[8] << "]\n";

    std::cout << "Gravity vector G(q)..." << std::flush;
    auto G = model.gravity_vector(q);
    std::cout << " G = [" << G[0] << ", " << G[1] << ", " << G[2] << "]\n";

    std::cout << "Forward dynamics (zero torque)..." << std::flush;
    auto ddq = model.forward_dynamics(q, dq, tau);
    std::cout << " ddq = [" << ddq[0] << ", " << ddq[1] << ", " << ddq[2] << "]\n";

    std::cout << "Forward dynamics (gravity comp)..." << std::flush;
    auto ddq_gc = model.forward_dynamics(q, dq, G);
    std::cout << " ddq = [" << ddq_gc[0] << ", " << ddq_gc[1] << ", " << ddq_gc[2] << "]\n";

    std::cout << "Forward kinematics..." << std::flush;
    auto fk = model.forward_kinematics(q);
    for (size_t i = 0; i < fk.size(); ++i) {
        std::cout << "  T_" << i << " pos = ["
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
