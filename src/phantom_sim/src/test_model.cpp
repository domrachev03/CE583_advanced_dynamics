#include "phantom_model/robot_model.hpp"
#include <iostream>
#include <cmath>

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: test_model <config_path>\n";
        return 1;
    }

    phantom_model::RobotModel model(argv[1]);

    auto get_pos = [](const std::array<double,16>& T) {
        return std::array<double,3>{T[12], T[13], T[14]};
    };
    auto mat_mul = [](const std::array<double,16>& A, const std::array<double,16>& B) {
        std::array<double,16> C{};
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                for (int k = 0; k < 4; ++k)
                    C[j*4+i] += A[k*4+i] * B[j*4+k];
        return C;
    };
    auto tcp_z = [&](const std::vector<double>& q) {
        auto fk = model.forward_kinematics(q);
        auto T06 = mat_mul(mat_mul(mat_mul(fk[0], fk[1]), fk[3]), fk[5]);
        return get_pos(T06)[2];
    };

    // Check key configurations
    std::cout << "TCP z at key positions:\n";
    std::cout << "  q=[0,0,0] (start):        z=" << tcp_z({0,0,0}) << "\n";
    std::cout << "  q=[0,0,pi/2] (should be low): z=" << tcp_z({0,0,M_PI/2}) << "\n";
    std::cout << "  q=[0,0,pi]:              z=" << tcp_z({0,0,M_PI}) << "\n";

    // Converged equilibrium from simulation
    std::cout << "\n  q=[0,-3.142,1.467] (converged): z="
              << tcp_z({0, -3.142, 1.467}) << "\n";

    // G at the converged point
    auto G = model.gravity_vector({0, -3.142, 1.467});
    std::cout << "  G = [" << G[0] << ", " << G[1] << ", " << G[2] << "] (should be ~0)\n";

    // G at q=0
    auto G0 = model.gravity_vector({0, 0, 0});
    std::cout << "  G(q=0) = [" << G0[0] << ", " << G0[1] << ", " << G0[2] << "]\n";

    // ddq at q=0
    auto ddq0 = model.forward_dynamics({0,0,0},{0,0,0},{0,0,0});
    std::cout << "  ddq(q=0) = [" << ddq0[0] << ", " << ddq0[1] << ", " << ddq0[2] << "]\n";

    // TCP z sweep for theta3 (with theta2=-pi to match converged state)
    std::cout << "\ntheta3 sweep (theta1=0, theta2=-pi):\n";
    for (double th3 = -M_PI; th3 <= M_PI; th3 += M_PI/6) {
        double z = tcp_z({0, -M_PI, th3});
        std::cout << "  theta3=" << th3 << "  z=" << z << "\n";
    }

    return 0;
}
