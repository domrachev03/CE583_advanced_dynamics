#include "phantom_model/robot_model.hpp"
#include <iostream>
#include <cmath>
#include <iomanip>
#include <vector>

// 3x3 matrix helpers (column-major like CasADi returns)
struct Mat3 {
    double data[9]; // column-major
    double& operator()(int r, int c) { return data[c*3+r]; }
    double  operator()(int r, int c) const { return data[c*3+r]; }
};

double dot3(const double* a, const double* b) {
    return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
}

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

    // Helper: compute potential energy from FK and link params
    auto compute_PE = [&](const std::vector<double>& q) {
        auto fk = model.forward_kinematics(q);
        // World transforms for each link chain
        auto T01 = fk[0];
        auto T02 = mat_mul(fk[0], fk[1]);
        auto T03 = mat_mul(fk[0], fk[2]);
        auto T04 = mat_mul(T02, fk[3]);
        auto T05 = mat_mul(T03, fk[4]);

        // COM positions in world (T_world * [com; 1])
        const auto& links = model.links();
        std::array<std::array<double,16>, 5> Tworld = {T01, T02, T03, T04, T05};
        double PE = 0;
        for (int i = 0; i < 5; ++i) {
            const auto& T = Tworld[i];
            const auto& com = links[i].com;
            // p_world = R * com + t  (column-major: R=[T[0..2], T[4..6], T[8..10]], t=[T[12],T[13],T[14]])
            double px = T[0]*com[0] + T[4]*com[1] + T[8]*com[2]  + T[12];
            double py = T[1]*com[0] + T[5]*com[1] + T[9]*com[2]  + T[13];
            double pz = T[2]*com[0] + T[6]*com[1] + T[10]*com[2] + T[14];
            PE += links[i].mass * 9.81 * pz;  // V = m*g*z
        }
        return PE;
    };

    // Helper: compute kinetic energy = 0.5 * dq^T * D(q) * dq
    auto compute_KE = [&](const std::vector<double>& q, const std::vector<double>& dq) {
        auto Dvec = model.inertia_matrix(q);
        Mat3 D;
        for (int i = 0; i < 9; ++i) D.data[i] = Dvec[i];
        double KE = 0;
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                KE += dq[r] * D(r, c) * dq[c];
        return 0.5 * KE;
    };

    std::cout << "=== Basic checks ===\n";
    std::cout << "TCP z at q=[0,0,0]: " << tcp_z({0,0,0}) << "\n";

    auto G0 = model.gravity_vector({0, 0, 0});
    std::cout << "G(q=0) = [" << G0[0] << ", " << G0[1] << ", " << G0[2] << "]\n";

    auto ddq0 = model.forward_dynamics({0,0,0},{0,0,0},{0,0,0});
    std::cout << "ddq(q=0) = [" << ddq0[0] << ", " << ddq0[1] << ", " << ddq0[2] << "]\n";

    // D matrix at q=0
    auto D0 = model.inertia_matrix({0, 0, 0});
    std::cout << "D(q=0) = [\n";
    for (int r = 0; r < 3; ++r) {
        std::cout << "  ";
        for (int c = 0; c < 3; ++c)
            std::cout << std::setw(14) << D0[c*3+r] << " ";
        std::cout << "\n";
    }
    std::cout << "]\n";

    // Verify D is theta1-independent
    auto D1 = model.inertia_matrix({1.0, 0, 0});
    auto D2 = model.inertia_matrix({2.5, 0, 0});
    double max_diff = 0;
    for (int i = 0; i < 9; ++i)
        max_diff = std::max(max_diff, std::abs(D0[i] - D1[i]));
    for (int i = 0; i < 9; ++i)
        max_diff = std::max(max_diff, std::abs(D0[i] - D2[i]));
    std::cout << "Max D difference across theta1 values: " << max_diff << "\n";

    // === Check Ddot - 2C skew-symmetry ===
    std::cout << "\n=== Ddot - 2C skew-symmetry check ===\n";
    std::vector<double> q_test = {0.0, -1.5, 0.8};
    std::vector<double> dq_test = {0.5, -2.0, 1.0};

    // Compute Ddot = sum_i (dD/dq_i) * dq_i  using finite differences
    double eps = 1e-7;
    auto D_at = [&](const std::vector<double>& q) { return model.inertia_matrix(q); };
    auto D_ref = D_at(q_test);

    Mat3 Ddot;
    for (int i = 0; i < 9; ++i) Ddot.data[i] = 0;

    for (int qi = 0; qi < 3; ++qi) {
        auto qp = q_test;
        qp[qi] += eps;
        auto Dp = D_at(qp);
        for (int i = 0; i < 9; ++i)
            Ddot.data[i] += (Dp[i] - D_ref[i]) / eps * dq_test[qi];
    }

    auto Cvec = model.coriolis_matrix(q_test, dq_test);
    Mat3 C_mat;
    for (int i = 0; i < 9; ++i) C_mat.data[i] = Cvec[i];

    std::cout << "S = Ddot - 2C:\n";
    Mat3 S;
    double max_skew = 0;
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            S(r, c) = Ddot(r, c) - 2.0 * C_mat(r, c);

    for (int r = 0; r < 3; ++r) {
        std::cout << "  ";
        for (int c = 0; c < 3; ++c)
            std::cout << std::setw(14) << S(r, c) << " ";
        std::cout << "\n";
    }

    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            max_skew = std::max(max_skew, std::abs(S(r, c) + S(c, r)));
    std::cout << "Max |S + S^T| (should be ~0): " << max_skew << "\n";

    // === Verify D*ddq + C*dq + G = tau ===
    std::cout << "\n=== Forward dynamics consistency check ===\n";
    std::vector<double> tau_test = {0.1, -0.05, 0.02};
    auto ddq_test = model.forward_dynamics(q_test, dq_test, tau_test);
    auto G_test = model.gravity_vector(q_test);

    std::cout << "Residual D*ddq + C*dq + G - tau (should be ~0):\n  [";
    for (int r = 0; r < 3; ++r) {
        double val = G_test[r] - tau_test[r];
        for (int c = 0; c < 3; ++c)
            val += D_ref[c*3+r] * ddq_test[c] + C_mat(r, c) * dq_test[c];
        std::cout << val << (r < 2 ? ", " : "");
    }
    std::cout << "]\n";

    // === Energy conservation simulation ===
    std::cout << "\n=== Simulation (RK4, 60s, dt=0.0002) ===\n";
    double dt = 0.0002;
    double sim_duration = 60.0;
    int total_steps = static_cast<int>(sim_duration / dt);
    std::vector<double> q = {0, 0, 0};
    std::vector<double> dq = {0, 0, 0};
    std::vector<double> tau = {0, 0, 0};

    double E0 = compute_KE(q, dq) + compute_PE(q);

    auto rk4_step = [&](std::vector<double>& q, std::vector<double>& dq,
                        const std::vector<double>& tau, double dt) {
        auto ddq1 = model.forward_dynamics(q, dq, tau);

        std::vector<double> q2(3), dq2(3);
        for (int i = 0; i < 3; ++i) {
            q2[i]  = q[i]  + dt/2 * dq[i];
            dq2[i] = dq[i] + dt/2 * ddq1[i];
        }
        auto ddq2 = model.forward_dynamics(q2, dq2, tau);

        std::vector<double> q3(3), dq3(3);
        for (int i = 0; i < 3; ++i) {
            q3[i]  = q[i]  + dt/2 * dq2[i];
            dq3[i] = dq[i] + dt/2 * ddq2[i];
        }
        auto ddq3 = model.forward_dynamics(q3, dq3, tau);

        std::vector<double> q4(3), dq4(3);
        for (int i = 0; i < 3; ++i) {
            q4[i]  = q[i]  + dt * dq3[i];
            dq4[i] = dq[i] + dt * ddq3[i];
        }
        auto ddq4 = model.forward_dynamics(q4, dq4, tau);

        for (int i = 0; i < 3; ++i) {
            q[i]  = q[i]  + dt/6 * (dq[i]  + 2*dq2[i]  + 2*dq3[i]  + dq4[i]);
            dq[i] = dq[i] + dt/6 * (ddq1[i] + 2*ddq2[i] + 2*ddq3[i] + ddq4[i]);
        }
    };

    int report_interval = total_steps / 30;  // ~30 reports
    std::cout << std::fixed << std::setprecision(4);
    for (int step = 0; step <= total_steps; ++step) {
        if (step % report_interval == 0) {
            double KE = compute_KE(q, dq);
            double PE = compute_PE(q);
            double E = KE + PE;
            double t = step * dt;
            double tz = tcp_z(q);
            std::cout << "t=" << std::setw(5) << t
                      << "  q=[" << std::setw(7) << q[0] << "," << std::setw(7) << q[1]
                      << "," << std::setw(7) << q[2] << "]"
                      << "  E=" << std::setw(8) << E
                      << "  dE=" << std::setw(10) << (E - E0)
                      << "  TCP_z=" << std::setw(7) << tz
                      << "\n";
        }
        if (step < total_steps) rk4_step(q, dq, tau, dt);
    }

    return 0;
}
