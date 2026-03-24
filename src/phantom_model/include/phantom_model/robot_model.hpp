#pragma once

#include <array>
#include <string>
#include <vector>

#include <casadi/casadi.hpp>

namespace phantom_model {

// ---------------------------------------------------------------------------
// Data read from the YAML config
// ---------------------------------------------------------------------------
struct LinkParams {
    std::string name;
    double mass;
    std::array<double, 3> com;                      // m, body frame
    std::array<std::array<double, 3>, 3> inertia;   // kg*m^2, body frame
};

struct KinematicParams {
    double base_height;       // m
    double upper_arm_offset;  // m
    double link4_x_offset;    // m
    double link5_x_offset;    // m
};

struct SimParams {
    double integration_dt;
    double publish_rate;
    bool   wait_for_input;
};

// ---------------------------------------------------------------------------
// RobotModel – builds CasADi symbolic dynamics once, then evaluates fast.
// ---------------------------------------------------------------------------
class RobotModel {
public:
    explicit RobotModel(const std::string& config_path);

    /// Forward dynamics:  ddq = D^{-1}(tau - C dq - G)
    std::vector<double> forward_dynamics(
        const std::vector<double>& q,
        const std::vector<double>& dq,
        const std::vector<double>& tau) const;

    /// Gravity vector  G(q)  (torque needed to hold position)
    std::vector<double> gravity_compensation(
        const std::vector<double>& q) const;

    /// Relative transforms for TF publishing (column-major 4x4, in mm).
    /// Order: T_01, T_12, T_13, T_24, T_35.
    std::vector<std::array<double, 16>> forward_kinematics(
        const std::vector<double>& q) const;

    int  ndof()   const { return 3; }
    int  nlinks() const { return 5; }

    const std::vector<LinkParams>& links()      const { return links_; }
    const KinematicParams&         kinematics()  const { return kin_params_; }
    const SimParams&               sim_params()  const { return sim_params_; }

private:
    void load_params(const std::string& config_path);
    void build_model();

    // ------ CasADi helpers (static – no state needed) -----------------------
    static casadi::SX rot_z(const casadi::SX& angle);
    static casadi::SX rot_x(double angle);
    static casadi::SX make_transform(const casadi::SX& R,
                                     const casadi::SX& t);
    static casadi::SX make_translation(const std::array<double, 3>& t);
    static casadi::SX make_inertia(
        const std::array<std::array<double, 3>, 3>& I);

    // ------ Compiled CasADi functions ---------------------------------------
    casadi::Function fwd_dyn_fn_;   // (q, dq, tau) -> ddq
    casadi::Function gravity_fn_;   // (q)          -> G
    casadi::Function fk_fn_;        // (q)          -> 5 x vec16

    // ------ Parameters loaded from YAML -------------------------------------
    std::vector<LinkParams> links_;
    KinematicParams kin_params_{};
    SimParams       sim_params_{};
    std::array<double, 3> gravity_{};
};

}  // namespace phantom_model
