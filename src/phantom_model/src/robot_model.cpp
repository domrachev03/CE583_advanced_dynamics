#include "phantom_model/robot_model.hpp"

#include <cmath>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace phantom_model {

using namespace casadi;

// ===========================================================================
// Construction
// ===========================================================================
RobotModel::RobotModel(const std::string& config_path) {
    load_params(config_path);
    build_model();
}

// ===========================================================================
// YAML loading
// ===========================================================================
void RobotModel::load_params(const std::string& config_path) {
    YAML::Node config = YAML::LoadFile(config_path);
    auto ph = config["phantom"];

    // Gravity
    for (int i = 0; i < 3; ++i)
        gravity_[i] = ph["gravity"][i].as<double>();

    // Kinematics
    auto kin = ph["kinematics"];
    kin_params_.base_height      = kin["base_height"].as<double>();
    kin_params_.upper_arm_offset = kin["upper_arm_offset"].as<double>();
    kin_params_.link4_x_offset   = kin["link4_x_offset"].as<double>();
    kin_params_.link5_x_offset   = kin["link5_x_offset"].as<double>();

    // Links
    links_.resize(5);
    for (int n = 0; n < 5; ++n) {
        std::string name = "link" + std::to_string(n + 1);
        links_[n].name = name;
        auto lk = ph["links"][name];
        links_[n].mass = lk["mass"].as<double>();
        for (int j = 0; j < 3; ++j)
            links_[n].com[j] = lk["com"][j].as<double>();
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                links_[n].inertia[r][c] = lk["inertia"][r][c].as<double>();
    }

}

// ===========================================================================
// Symbolic helpers
// ===========================================================================
SX RobotModel::rot_z(const SX& angle) {
    SX R = SX::zeros(3, 3);
    R(0, 0) =  cos(angle);  R(0, 1) = -sin(angle);
    R(1, 0) =  sin(angle);  R(1, 1) =  cos(angle);
    R(2, 2) =  1;
    return R;
}

SX RobotModel::rot_x(double angle) {
    double ca = std::cos(angle), sa = std::sin(angle);
    SX R = SX::zeros(3, 3);
    R(0, 0) = 1;
    R(1, 1) = ca;   R(1, 2) = -sa;
    R(2, 1) = sa;   R(2, 2) =  ca;
    return R;
}

SX RobotModel::make_transform(const SX& R, const SX& t) {
    SX T = SX::zeros(4, 4);
    T(Slice(0, 3), Slice(0, 3)) = R;
    T(Slice(0, 3), 3) = t;
    T(3, 3) = 1;
    return T;
}

SX RobotModel::make_translation(const std::array<double, 3>& v) {
    return SX::vertcat({v[0], v[1], v[2]});
}

SX RobotModel::make_inertia(
    const std::array<std::array<double, 3>, 3>& I) {
    SX M = SX::zeros(3, 3);
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            M(r, c) = I[r][c];
    return M;
}

// ===========================================================================
// Build the full symbolic model (called once)
// ===========================================================================
void RobotModel::build_model() {
    // ---- symbolic state ---------------------------------------------------
    SX q   = SX::sym("q", 3);
    SX dq  = SX::sym("dq", 3);
    SX tau = SX::sym("tau", 3);

    SX th1 = q(0), th2 = q(1), th3 = q(2);

    // ---- kinematic transforms (match reference MATLAB) --------------------
    // T_01 : base → link1   Rz(θ1),  tz = base_height
    SX T_01 = make_transform(
        rot_z(th1),
        SX::vertcat({0, 0, kin_params_.base_height}));

    // T_12 : link1 → link2  Rx(-π/2)·Rz(θ2−π/2),  tz = upper_arm_offset
    SX T_12 = make_transform(
        mtimes(rot_x(-M_PI / 2), rot_z(th2 - M_PI / 2)),
        SX::vertcat({0, 0, kin_params_.upper_arm_offset}));

    // T_13 : link1 → link3  Rx(-π/2)·Rz(θ3),  tz = upper_arm_offset
    SX T_13 = make_transform(
        mtimes(rot_x(-M_PI / 2), rot_z(th3)),
        SX::vertcat({0, 0, kin_params_.upper_arm_offset}));

    // T_24 : link2 → link4  Rz(π/2−θ2+θ3),  tx = link4_x_offset
    SX T_24 = make_transform(
        rot_z(M_PI / 2 - th2 + th3),
        SX::vertcat({kin_params_.link4_x_offset, 0, 0}));

    // T_35 : link3 → link5  Rz(π/2+θ2−θ3),  tx = link5_x_offset
    SX T_35 = make_transform(
        rot_z(M_PI / 2 + th2 - th3),
        SX::vertcat({kin_params_.link5_x_offset, 0, 0}));

    // ---- COM transforms (pure translation in body frame) ------------------
    std::vector<SX> T_com(5);
    for (int i = 0; i < 5; ++i)
        T_com[i] = make_transform(SX::eye(3), make_translation(links_[i].com));

    // ---- World-frame COM transforms ---------------------------------------
    SX T_01com = mtimes(T_01, T_com[0]);
    SX T_02com = mtimes(mtimes(T_01, T_12), T_com[1]);
    SX T_03com = mtimes(mtimes(T_01, T_13), T_com[2]);
    SX T_04com = mtimes(mtimes(mtimes(T_01, T_12), T_24), T_com[3]);
    SX T_05com = mtimes(mtimes(mtimes(T_01, T_13), T_35), T_com[4]);

    std::vector<SX> Tcom = {T_01com, T_02com, T_03com, T_04com, T_05com};

    // ---- COM positions  (3×1 each) ---------------------------------------
    std::vector<SX> Pc(5);
    for (int i = 0; i < 5; ++i)
        Pc[i] = Tcom[i](Slice(0, 3), 3);

    // ---- Linear-velocity Jacobians  Jvc = ∂Pc/∂q  (3×3 each) ------------
    std::vector<SX> Jvc(5);
    for (int i = 0; i < 5; ++i)
        Jvc[i] = jacobian(Pc[i], q);

    // ---- Angular-velocity Jacobians (Phantom parallel-linkage coupling) ---
    //  link1 : joint1
    //  link2 : joint1, joint2
    //  link3 : joint1,         joint3
    //  link4 : joint1,         joint3   (4-bar)
    //  link5 : joint1, joint2           (4-bar)
    SX z01 = T_01com(Slice(0, 3), 2);   // z-axis of frame 1 in world
    SX z02 = T_02com(Slice(0, 3), 2);   // z-axis of frame 2 in world
    SX z03 = T_03com(Slice(0, 3), 2);   // z-axis of frame 3 in world
    SX z0  = SX::zeros(3, 1);

    std::vector<SX> Jw(5);
    Jw[0] = SX::horzcat({z01,  z0,  z0 });
    Jw[1] = SX::horzcat({z01, z02,  z0 });
    Jw[2] = SX::horzcat({z01,  z0, z03});
    Jw[3] = SX::horzcat({z01,  z0, z03});   // 4-bar: link4 ← joint3
    Jw[4] = SX::horzcat({z01, z02,  z0 });  // 4-bar: link5 ← joint2

    // ---- Inertia tensors in world frame  R·I_body·Rᵀ ---------------------
    std::vector<SX> Iw(5);
    for (int i = 0; i < 5; ++i) {
        SX R      = Tcom[i](Slice(0, 3), Slice(0, 3));
        SX Ibody  = make_inertia(links_[i].inertia);
        Iw[i]     = mtimes(mtimes(R, Ibody), R.T());
    }

    // ---- Generalised inertia matrix  D(q)  --------------------------------
    SX D = SX::zeros(3, 3);
    for (int i = 0; i < 5; ++i) {
        D = D + links_[i].mass * mtimes(Jvc[i].T(), Jvc[i])
              + mtimes(mtimes(Jw[i].T(), Iw[i]), Jw[i]);
    }

    // ---- Coriolis / centrifugal matrix  C(q, dq)  via Christoffel symbols -
    SX C = SX::zeros(3, 3);
    for (int j = 0; j < 3; ++j) {
        for (int k = 0; k < 3; ++k) {
            SX cjk = SX::zeros(1, 1);
            for (int i = 0; i < 3; ++i) {
                cjk = cjk
                    + (jacobian(D(k, j), q(i))
                     + jacobian(D(k, i), q(j))
                     - jacobian(D(i, j), q(k))) / 2.0 * dq(i);
            }
            C(j, k) = cjk;
        }
    }

    // ---- Gravity vector  G(q) = ∂V/∂q  -----------------------------------
    SX gvec = SX::vertcat({gravity_[0], gravity_[1], gravity_[2]});
    SX PE   = SX::zeros(1, 1);
    for (int i = 0; i < 5; ++i)
        PE = PE + links_[i].mass * dot(Pc[i], gvec);
    SX G = gradient(PE, q);

    // ---- Forward dynamics  ddq = D⁻¹(τ − C·dq − G)  ---------------------
    SX ddq = solve(D, tau - mtimes(C, dq) - G);

    fwd_dyn_fn_ = Function("fwd_dyn",
        {q, dq, tau}, {ddq},
        {"q", "dq", "tau"}, {"ddq"});

    inertia_fn_ = Function("inertia",
        {q}, {D},
        {"q"}, {"D"});

    coriolis_fn_ = Function("coriolis",
        {q, dq}, {C},
        {"q", "dq"}, {"C"});

    gravity_fn_ = Function("gravity",
        {q}, {G},
        {"q"}, {"G"});

    // ---- TCP frame (parallelogram closure: link4 → link6) -----------------
    // The 4th vertex of the parallelogram sits at +link5_x_offset from link4
    // (by the equal-opposite-sides property of a parallelogram).
    SX T_46 = make_transform(SX::eye(3),
        SX::vertcat({kin_params_.link5_x_offset, 0, 0}));

    // ---- Forward kinematics (relative transforms for TF) ------------------
    fk_fn_ = Function("fk", {q},
        {reshape(T_01, 16, 1),
         reshape(T_12, 16, 1),
         reshape(T_13, 16, 1),
         reshape(T_24, 16, 1),
         reshape(T_35, 16, 1),
         reshape(T_46, 16, 1)},
        {"q"},
        {"T_01", "T_12", "T_13", "T_24", "T_35", "T_46"});
}

// ===========================================================================
// Public evaluation wrappers
// ===========================================================================
std::vector<double> RobotModel::forward_dynamics(
    const std::vector<double>& q,
    const std::vector<double>& dq,
    const std::vector<double>& tau) const {
    std::vector<DM> args = {DM(q), DM(dq), DM(tau)};
    auto res = fwd_dyn_fn_(args);
    return res.at(0).nonzeros();
}

std::vector<double> RobotModel::inertia_matrix(
    const std::vector<double>& q) const {
    std::vector<DM> args = {DM(q)};
    auto res = inertia_fn_(args);
    return res.at(0).nonzeros();
}

std::vector<double> RobotModel::coriolis_matrix(
    const std::vector<double>& q,
    const std::vector<double>& dq) const {
    std::vector<DM> args = {DM(q), DM(dq)};
    auto res = coriolis_fn_(args);
    return res.at(0).nonzeros();
}

std::vector<double> RobotModel::gravity_vector(
    const std::vector<double>& q) const {
    std::vector<DM> args = {DM(q)};
    auto res = gravity_fn_(args);
    return res.at(0).nonzeros();
}

std::vector<std::array<double, 16>> RobotModel::forward_kinematics(
    const std::vector<double>& q) const {
    std::vector<DM> args = {DM(q)};
    auto res = fk_fn_(args);
    const int nt = static_cast<int>(res.size());
    std::vector<std::array<double, 16>> out(nt);
    for (int i = 0; i < nt; ++i) {
        // densify: CasADi may mark constant entries as structural zeros
        DM dense = DM::densify(res.at(i));
        auto v = dense.nonzeros();
        std::copy(v.begin(), v.end(), out[i].begin());
    }
    return out;
}

}  // namespace phantom_model
