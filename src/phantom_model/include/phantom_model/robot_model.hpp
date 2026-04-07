#pragma once

#include <array>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <casadi/casadi.hpp>

namespace phantom_model {

// ---------------------------------------------------------------------------
// Data read from the YAML config
// ---------------------------------------------------------------------------
struct LinkParams {
    std::string     name;
    double          mass;       // kg
    Eigen::Vector3d com;        // m, body frame
    Eigen::Matrix3d inertia;    // kg*m^2, body frame
};

struct KinematicParams {
    double base_height;       // m
    double upper_arm_offset;  // m
    double link4_x_offset;    // m
    double link5_x_offset;    // m
};

// ---------------------------------------------------------------------------
// Parametric model errors applied at YAML-load time (before build_model).
//
// Purpose: let a controller build a RobotModel whose parameters differ from
// the "ground truth" values used by the simulator, so robust/adaptive control
// laws can be stress-tested against a known parametric mismatch.
//
// Only link masses are scaled.  Inertia tensors and the gravitational field
// vector are left untouched.  Because G(q) is derived from scaled masses via
// V = -sum_i m_i * p_i^T * g, it scales linearly with mass as a natural
// consequence — no separate "gravity scale" is needed or exposed.
//
// Physical constraints (all preserved by construction for any positive
// scales): D ≻ 0 (sum of PD contributions each weighted by a positive scalar),
// D = D^T, Ḋ - 2C skew-symmetric (C derived from scaled D via the same
// Christoffel symbols), G = ∂V/∂q (V rebuilt from scaled masses).
// ---------------------------------------------------------------------------
struct ModelErrors {
    /// Uniform mass scale applied to every link.  Must be > 0.
    double scalar_scale = 1.0;

    /// Per-link mass scale (one entry per link, 5 for this robot).  Composes
    /// multiplicatively with scalar_scale:
    ///     effective_m_i = scalar_scale * per_link_scale[i] * yaml_mass_i
    /// Each entry must be > 0.
    std::array<double, 5> per_link_scale = {1.0, 1.0, 1.0, 1.0, 1.0};
};

// ---------------------------------------------------------------------------
// RobotModel – builds CasADi symbolic dynamics once, then evaluates fast.
// ---------------------------------------------------------------------------
class RobotModel {
public:
    /// Build a model from a YAML config with ground-truth parameters.
    explicit RobotModel(const std::string& config_path);

    /// Build a model from a YAML config, applying mass scaling for
    /// parametric-error testing.  See ModelErrors docs above for semantics.
    RobotModel(const std::string& config_path, const ModelErrors& errors);

    /// Forward dynamics:  ddq = D^{-1}(tau - C dq - G)
    Eigen::Vector3d forward_dynamics(
        const Eigen::Vector3d& q,
        const Eigen::Vector3d& dq,
        const Eigen::Vector3d& tau) const;

    /// Inertia matrix  D(q)
    Eigen::Matrix3d inertia_matrix(const Eigen::Vector3d& q) const;

    /// Coriolis/centrifugal matrix  C(q, dq)
    Eigen::Matrix3d coriolis_matrix(const Eigen::Vector3d& q,
                                    const Eigen::Vector3d& dq) const;

    /// Gravity vector  G(q)
    Eigen::Vector3d gravity_vector(const Eigen::Vector3d& q) const;

    /// Relative transforms for TF publishing.
    /// Order: T_01, T_12, T_13, T_24, T_35, T_46.
    std::array<Eigen::Matrix4d, 6> forward_kinematics(
        const Eigen::Vector3d& q) const;

    /// TCP (link6) geometric Jacobian, 6x3.
    /// Top 3 rows: linear velocity mapping  (v_tcp    = J_v * dq)
    /// Bot 3 rows: angular velocity mapping (omega_4  = J_w * dq)
    /// Because the TCP rides on link4 via the parallelogram closure, its
    /// angular velocity equals link4's, which is driven by joints 1 and 3
    /// only (joint 2 has zero contribution).
    Eigen::Matrix<double, 6, 3> tcp_jacobian(const Eigen::Vector3d& q) const;

    /// Time derivative of tcp_jacobian at (q, dq), i.e.
    ///     d/dt J(q(t))  =  (dJ/dq) * dq
    /// computed symbolically via CasADi's jtimes (forward-mode directional
    /// derivative), so no finite differencing is involved.
    Eigen::Matrix<double, 6, 3> tcp_jacobian_dot(
        const Eigen::Vector3d& q,
        const Eigen::Vector3d& dq) const;

    int  ndof()   const { return 3; }
    int  nlinks() const { return 5; }

    const std::vector<LinkParams>& links()      const { return links_; }
    const KinematicParams&         kinematics()  const { return kin_params_; }

private:
    void load_params(const std::string& config_path, const ModelErrors& errors);
    void build_model();

    // ------ CasADi helpers (static – no state needed) -----------------------
    static casadi::SX rot_z(const casadi::SX& angle);
    static casadi::SX rot_x(double angle);
    static casadi::SX make_transform(const casadi::SX& R,
                                     const casadi::SX& t);
    static casadi::SX make_translation(const Eigen::Vector3d& t);
    static casadi::SX make_inertia(const Eigen::Matrix3d& I);

    // ------ Compiled CasADi functions ---------------------------------------
    casadi::Function fwd_dyn_fn_;       // (q, dq, tau) -> ddq
    casadi::Function inertia_fn_;       // (q)          -> D (3x3)
    casadi::Function coriolis_fn_;      // (q, dq)      -> C (3x3)
    casadi::Function gravity_fn_;       // (q)          -> G (3x1)
    casadi::Function fk_fn_;            // (q)          -> 6 x (4x4)
    casadi::Function tcp_jac_fn_;       // (q)          -> J_tcp (6x3)
    casadi::Function tcp_jac_dot_fn_;   // (q, dq)      -> J_tcp_dot (6x3)

    // ------ Parameters loaded from YAML -------------------------------------
    std::vector<LinkParams> links_;
    KinematicParams kin_params_{};
    Eigen::Vector3d gravity_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d damping_{Eigen::Vector3d::Zero()};  // per-joint viscous damping
};

}  // namespace phantom_model
