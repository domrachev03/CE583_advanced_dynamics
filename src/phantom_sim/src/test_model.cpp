#include "phantom_model/robot_model.hpp"

#include <iomanip>
#include <iostream>

#include <Eigen/Dense>

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: test_model <config_path>\n";
        return 1;
    }

    phantom_model::RobotModel model(argv[1]);

    // TCP z-height via FK composition.
    auto tcp_z = [&](const Eigen::Vector3d& q) {
        auto fk = model.forward_kinematics(q);
        Eigen::Matrix4d T06 = fk[0] * fk[1] * fk[3] * fk[5];
        return T06(2, 3);
    };

    // Potential energy from FK + link params: V = sum_i m_i * g * z_com_i
    auto compute_PE = [&](const Eigen::Vector3d& q) {
        auto fk = model.forward_kinematics(q);
        // World transforms for each link's parent frame
        Eigen::Matrix4d T01 = fk[0];
        Eigen::Matrix4d T02 = fk[0] * fk[1];
        Eigen::Matrix4d T03 = fk[0] * fk[2];
        Eigen::Matrix4d T04 = T02    * fk[3];
        Eigen::Matrix4d T05 = T03    * fk[4];
        const std::array<Eigen::Matrix4d, 5> Tworld = {T01, T02, T03, T04, T05};

        const auto& links = model.links();
        double PE = 0;
        for (int i = 0; i < 5; ++i) {
            const Eigen::Matrix4d& T = Tworld[i];
            // p_world = R * com + t
            const Eigen::Vector3d p_world =
                T.block<3, 3>(0, 0) * links[i].com + T.block<3, 1>(0, 3);
            PE += links[i].mass * 9.81 * p_world.z();
        }
        return PE;
    };

    // Kinetic energy: KE = 0.5 * dq^T * D(q) * dq
    auto compute_KE = [&](const Eigen::Vector3d& q, const Eigen::Vector3d& dq) {
        Eigen::Matrix3d D = model.inertia_matrix(q);
        return 0.5 * dq.transpose() * D * dq;
    };

    std::cout << "=== Basic checks ===\n";
    std::cout << "TCP z at q=[0,0,0]: " << tcp_z(Eigen::Vector3d::Zero()) << "\n";

    Eigen::Vector3d G0 = model.gravity_vector(Eigen::Vector3d::Zero());
    std::cout << "G(q=0) = [" << G0(0) << ", " << G0(1) << ", " << G0(2) << "]\n";

    Eigen::Vector3d ddq0 = model.forward_dynamics(
        Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
    std::cout << "ddq(q=0) = [" << ddq0(0) << ", " << ddq0(1) << ", " << ddq0(2) << "]\n";

    // D matrix at q=0
    Eigen::Matrix3d D0 = model.inertia_matrix(Eigen::Vector3d::Zero());
    std::cout << "D(q=0) = [\n";
    for (int r = 0; r < 3; ++r) {
        std::cout << "  ";
        for (int c = 0; c < 3; ++c)
            std::cout << std::setw(14) << D0(r, c) << " ";
        std::cout << "\n";
    }
    std::cout << "]\n";

    // Verify D is theta1-independent
    Eigen::Matrix3d D1 = model.inertia_matrix(Eigen::Vector3d(1.0, 0, 0));
    Eigen::Matrix3d D2 = model.inertia_matrix(Eigen::Vector3d(2.5, 0, 0));
    double max_diff = std::max((D0 - D1).cwiseAbs().maxCoeff(),
                               (D0 - D2).cwiseAbs().maxCoeff());
    std::cout << "Max D difference across theta1 values: " << max_diff << "\n";

    // === Check Ddot - 2C skew-symmetry ===
    std::cout << "\n=== Ddot - 2C skew-symmetry check ===\n";
    Eigen::Vector3d q_test (0.0, -1.5, 0.8);
    Eigen::Vector3d dq_test(0.5, -2.0, 1.0);

    // Ddot = sum_i (dD/dq_i) * dq_i via finite differences
    const double eps = 1e-7;
    Eigen::Matrix3d D_ref = model.inertia_matrix(q_test);
    Eigen::Matrix3d Ddot  = Eigen::Matrix3d::Zero();
    for (int qi = 0; qi < 3; ++qi) {
        Eigen::Vector3d qp = q_test;
        qp(qi) += eps;
        Ddot += (model.inertia_matrix(qp) - D_ref) / eps * dq_test(qi);
    }

    Eigen::Matrix3d C_mat = model.coriolis_matrix(q_test, dq_test);
    Eigen::Matrix3d S     = Ddot - 2.0 * C_mat;

    std::cout << "S = Ddot - 2C:\n";
    for (int r = 0; r < 3; ++r) {
        std::cout << "  ";
        for (int c = 0; c < 3; ++c)
            std::cout << std::setw(14) << S(r, c) << " ";
        std::cout << "\n";
    }
    const double max_skew = (S + S.transpose()).cwiseAbs().maxCoeff();
    std::cout << "Max |S + S^T| (should be ~0): " << max_skew << "\n";

    // === TCP Jacobian checks ===
    // (1) J * dq should equal d/dt p_tcp computed by central finite
    //     difference on forward_kinematics at (q, dq).
    // (2) tcp_jacobian_dot(q, dq) should equal the central finite difference
    //     of tcp_jacobian along the dq direction.
    std::cout << "\n=== TCP Jacobian checks ===\n";
    {
        auto p_tcp_of = [&](const Eigen::Vector3d& qv) {
            auto fk = model.forward_kinematics(qv);
            Eigen::Matrix4d T06 = fk[0] * fk[1] * fk[3] * fk[5];
            return Eigen::Vector3d(T06.block<3, 1>(0, 3));
        };

        const double h = 1e-6;
        Eigen::Matrix<double, 6, 3> J = model.tcp_jacobian(q_test);
        Eigen::Vector3d v_analytic = J.block<3, 3>(0, 0) * dq_test;

        Eigen::Vector3d p_plus  = p_tcp_of(q_test + h * dq_test);
        Eigen::Vector3d p_minus = p_tcp_of(q_test - h * dq_test);
        Eigen::Vector3d v_fd    = (p_plus - p_minus) / (2 * h);

        std::cout << "  |J*dq − v_fd| = "
                  << (v_analytic - v_fd).cwiseAbs().maxCoeff() << "\n";

        Eigen::Matrix<double, 6, 3> J_dot_analytic =
            model.tcp_jacobian_dot(q_test, dq_test);
        Eigen::Matrix<double, 6, 3> J_plus  = model.tcp_jacobian(q_test + h * dq_test);
        Eigen::Matrix<double, 6, 3> J_minus = model.tcp_jacobian(q_test - h * dq_test);
        Eigen::Matrix<double, 6, 3> J_dot_fd = (J_plus - J_minus) / (2 * h);

        std::cout << "  |J_dot − J_dot_fd| = "
                  << (J_dot_analytic - J_dot_fd).cwiseAbs().maxCoeff() << "\n";
    }

    // === ModelErrors checks ===
    // Since only masses are scaled (not inertia tensors), the inertia
    // matrix splits as  D = D_trans(m) + D_rot(I)  where only the first
    // half has m_i as a coefficient.  So D does NOT scale linearly with k
    // under mass-only perturbation.  G, on the other hand, has V = -Σ m_i
    // p_i^T g, which does scale linearly, so G(k) = k·G(1) exactly.
    //
    // Independent sanity checks that survive the mass-only convention:
    //   (1) Default ModelErrors{} produces bit-identical D, C, G.
    //   (2) G scales linearly with scalar_scale.
    //   (3) Scaled model still satisfies D*ddq + C*dq + G = tau at machine
    //       precision (physical consistency of the rebuilt symbolic model).
    //   (4) Scaled D is still symmetric and its determinant is positive
    //       (necessary condition for PD).
    std::cout << "\n=== ModelErrors checks ===\n";
    {
        // (1) default ModelErrors{} → bit-identical
        phantom_model::RobotModel model_default(argv[1], phantom_model::ModelErrors{});
        double d_default =
            (model_default.inertia_matrix(q_test) - D_ref).cwiseAbs().maxCoeff();
        std::cout << "  default ModelErrors{}: |D - D_ref| = " << d_default << "\n";

        // (2) G(k) = k·G(1)
        phantom_model::ModelErrors errors;
        errors.scalar_scale = 2.0;
        phantom_model::RobotModel model_2x(argv[1], errors);

        Eigen::Vector3d G_ref_1x = model.gravity_vector(q_test);
        Eigen::Vector3d G_ref_2x = model_2x.gravity_vector(q_test);
        double dG = (G_ref_2x - 2.0 * G_ref_1x).cwiseAbs().maxCoeff();
        std::cout << "  G scaling: |G(k=2) - 2·G(k=1)| = " << dG << "\n";

        // (3) forward_dynamics consistency at the scaled model
        Eigen::Matrix3d D_2x   = model_2x.inertia_matrix(q_test);
        Eigen::Matrix3d C_2x   = model_2x.coriolis_matrix(q_test, dq_test);
        Eigen::Vector3d tau_2x = Eigen::Vector3d(0.1, -0.05, 0.02);
        Eigen::Vector3d ddq_2x = model_2x.forward_dynamics(q_test, dq_test, tau_2x);
        Eigen::Vector3d res_2x = D_2x * ddq_2x + C_2x * dq_test + G_ref_2x - tau_2x;
        std::cout << "  forward-dyn residual at k=2: "
                  << res_2x.cwiseAbs().maxCoeff() << "\n";

        // (4) Scaled D symmetry + det > 0
        double sym_err = (D_2x - D_2x.transpose()).cwiseAbs().maxCoeff();
        double det_D_2x = D_2x.determinant();
        std::cout << "  scaled D symmetry err: " << sym_err
                  << ", det(D) = " << det_D_2x
                  << (det_D_2x > 0 ? " (OK)" : " (FAIL)") << "\n";
    }

    // === Verify D*ddq + C*dq + G = tau ===
    std::cout << "\n=== Forward dynamics consistency check ===\n";
    Eigen::Vector3d tau_test(0.1, -0.05, 0.02);
    Eigen::Vector3d ddq_test = model.forward_dynamics(q_test, dq_test, tau_test);
    Eigen::Vector3d G_test   = model.gravity_vector(q_test);
    Eigen::Vector3d residual = D_ref * ddq_test + C_mat * dq_test + G_test - tau_test;

    std::cout << "Residual D*ddq + C*dq + G - tau (should be ~0):\n  ["
              << residual(0) << ", " << residual(1) << ", " << residual(2) << "]\n";

    // === Energy conservation simulation ===
    std::cout << "\n=== Simulation (RK4, 60s, dt=0.0002) ===\n";
    const double dt = 0.0002;
    const double sim_duration = 60.0;
    const int total_steps = static_cast<int>(sim_duration / dt);
    Eigen::Vector3d q   = Eigen::Vector3d::Zero();
    Eigen::Vector3d dq  = Eigen::Vector3d::Zero();
    Eigen::Vector3d tau = Eigen::Vector3d::Zero();

    const double E0 = compute_KE(q, dq) + compute_PE(q);

    // Classical RK4 (same as simulator_node.cpp::rk4_step).
    auto rk4_step = [&](Eigen::Vector3d& q_io,
                        Eigen::Vector3d& dq_io,
                        const Eigen::Vector3d& tau_in,
                        double dt_in) {
        auto eval = [&](const Eigen::Vector3d& qv, const Eigen::Vector3d& dqv) {
            return model.forward_dynamics(qv, dqv, tau_in);
        };

        Eigen::Vector3d k1 = eval(q_io, dq_io);

        Eigen::Vector3d q2  = q_io  + (dt_in / 2) * dq_io;
        Eigen::Vector3d dq2 = dq_io + (dt_in / 2) * k1;
        Eigen::Vector3d k2  = eval(q2, dq2);

        Eigen::Vector3d q3  = q_io  + (dt_in / 2) * dq2;
        Eigen::Vector3d dq3 = dq_io + (dt_in / 2) * k2;
        Eigen::Vector3d k3  = eval(q3, dq3);

        Eigen::Vector3d q4  = q_io  + dt_in * dq3;
        Eigen::Vector3d dq4 = dq_io + dt_in * k3;
        Eigen::Vector3d k4  = eval(q4, dq4);

        q_io  += (dt_in / 6) * (dq_io + 2 * dq2 + 2 * dq3 + dq4);
        dq_io += (dt_in / 6) * (k1    + 2 * k2  + 2 * k3  + k4);
    };

    const int report_interval = total_steps / 30;  // ~30 reports
    std::cout << std::fixed << std::setprecision(4);
    for (int step = 0; step <= total_steps; ++step) {
        if (step % report_interval == 0) {
            double KE = compute_KE(q, dq);
            double PE = compute_PE(q);
            double E  = KE + PE;
            double t  = step * dt;
            double tz = tcp_z(q);
            std::cout << "t=" << std::setw(5) << t
                      << "  q=[" << std::setw(7) << q(0) << "," << std::setw(7) << q(1)
                      << "," << std::setw(7) << q(2) << "]"
                      << "  E=" << std::setw(8) << E
                      << "  dE=" << std::setw(10) << (E - E0)
                      << "  TCP_z=" << std::setw(7) << tz
                      << "\n";
        }
        if (step < total_steps) rk4_step(q, dq, tau, dt);
    }

    return 0;
}
