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
// RobotModel – builds CasADi symbolic dynamics once, then evaluates fast.
// ---------------------------------------------------------------------------
class RobotModel {
public:
    explicit RobotModel(const std::string& config_path);

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

    int  ndof()   const { return 3; }
    int  nlinks() const { return 5; }

    const std::vector<LinkParams>& links()      const { return links_; }
    const KinematicParams&         kinematics()  const { return kin_params_; }

private:
    void load_params(const std::string& config_path);
    void build_model();

    // ------ CasADi helpers (static – no state needed) -----------------------
    static casadi::SX rot_z(const casadi::SX& angle);
    static casadi::SX rot_x(double angle);
    static casadi::SX make_transform(const casadi::SX& R,
                                     const casadi::SX& t);
    static casadi::SX make_translation(const Eigen::Vector3d& t);
    static casadi::SX make_inertia(const Eigen::Matrix3d& I);

    // ------ Compiled CasADi functions ---------------------------------------
    casadi::Function fwd_dyn_fn_;   // (q, dq, tau) -> ddq
    casadi::Function inertia_fn_;   // (q)          -> D (3x3)
    casadi::Function coriolis_fn_;  // (q, dq)      -> C (3x3)
    casadi::Function gravity_fn_;   // (q)          -> G (3x1)
    casadi::Function fk_fn_;        // (q)          -> 6 x (4x4)

    // ------ Parameters loaded from YAML -------------------------------------
    std::vector<LinkParams> links_;
    KinematicParams kin_params_{};
    Eigen::Vector3d gravity_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d damping_{Eigen::Vector3d::Zero()};  // per-joint viscous damping
};

}  // namespace phantom_model
