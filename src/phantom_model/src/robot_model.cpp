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
        gravity_(i) = ph["gravity"][i].as<double>();

    // Damping (optional, defaults to zero). The robot is fixed at 3 DOF,
    // so damping_ is always a Vector3d.
    damping_.setZero();
    if (ph["damping"]) {
        for (int i = 0; i < 3; ++i)
            damping_(i) = ph["damping"][i].as<double>();
    }

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
            links_[n].com(j) = lk["com"][j].as<double>();
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                links_[n].inertia(r, c) = lk["inertia"][r][c].as<double>();
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

SX RobotModel::make_translation(const Eigen::Vector3d& v) {
    return SX::vertcat({v(0), v(1), v(2)});
}

SX RobotModel::make_inertia(const Eigen::Matrix3d& I) {
    SX M = SX::zeros(3, 3);
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            M(r, c) = I(r, c);
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

    // Joint 1 rotates the entire mechanism about the gravity (z) axis, so
    // D(q) is analytically independent of θ1.  CasADi cannot simplify
    // sin²(θ1)+cos²(θ1)=1, which leaves spurious θ1-dependence that breaks
    // the skew-symmetry of Ḋ−2C and injects energy numerically.
    // Setting θ1=0 eliminates the spurious terms; the result is valid for
    // all θ1 because the true D does not depend on it.
    D = SX::substitute(D, th1, SX(0));
    D = SX::simplify(D);

    // ---- Coriolis / centrifugal matrix  C(q, dq)  via Christoffel symbols -
    SX C = SX::zeros(3, 3);
    for (int j = 0; j < 3; ++j) {
        for (int k = 0; k < 3; ++k) {
            SX ckj = SX::zeros(1, 1);
            for (int i = 0; i < 3; ++i) {
                ckj = ckj
                    + (jacobian(D(k, j), q(i))
                     + jacobian(D(k, i), q(j))
                     - jacobian(D(i, j), q(k))) / 2.0 * dq(i);
            }
            C(k, j) = ckj;
        }
    }

    // ---- Gravity vector  G(q) = ∂V/∂q  -----------------------------------
    // V = -m·dot(P, g) so that V = +m·9.81·z (increases with height).
    // With g=[0,0,-9.81]: V = -m·(-9.81)·z = +m·9.81·z  ✓
    SX gvec = SX::vertcat({gravity_[0], gravity_[1], gravity_[2]});
    SX PE   = SX::zeros(1, 1);
    for (int i = 0; i < 5; ++i)
        PE = PE - links_[i].mass * dot(Pc[i], gvec);
    SX G = gradient(PE, q);

    // Same θ1-independence reasoning as for D (gravity along z cannot
    // create torque about a z-rotation).
    G = SX::substitute(G, th1, SX(0));
    G = SX::simplify(G);

    // ---- Joint damping  B·dq  (diagonal, optional) -------------------------
    SX B = SX::zeros(3, 3);
    for (int i = 0; i < 3; ++i)
        B(i, i) = damping_[i];

    // ---- Forward dynamics  ddq = D⁻¹(τ − C·dq − G − B·dq)  --------------
    SX ddq = solve(D, tau - mtimes(C, dq) - G - mtimes(B, dq));

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
// CasADi <-> Eigen bridge.
//
// Both CasADi DM (dense) and Eigen (by default) store data in column-major
// order, so we can Eigen::Map directly over DM::ptr() after densifying.
// Densification is necessary because CasADi may mark structurally-zero
// entries as missing; without it, ptr() would not cover every cell.
namespace {

inline DM to_dm(const Eigen::Vector3d& v) {
    return DM(std::vector<double>{v(0), v(1), v(2)});
}

inline Eigen::Vector3d to_vec3(const DM& dm) {
    DM dense = DM::densify(dm);
    return Eigen::Map<const Eigen::Vector3d>(dense.ptr());
}

inline Eigen::Matrix3d to_mat3(const DM& dm) {
    DM dense = DM::densify(dm);
    return Eigen::Map<const Eigen::Matrix3d>(dense.ptr());
}

inline Eigen::Matrix4d to_mat4(const DM& dm) {
    DM dense = DM::densify(dm);
    return Eigen::Map<const Eigen::Matrix4d>(dense.ptr());
}

}  // namespace

Eigen::Vector3d RobotModel::forward_dynamics(
    const Eigen::Vector3d& q,
    const Eigen::Vector3d& dq,
    const Eigen::Vector3d& tau) const {
    std::vector<DM> args = {to_dm(q), to_dm(dq), to_dm(tau)};
    auto res = fwd_dyn_fn_(args);
    return to_vec3(res.at(0));
}

Eigen::Matrix3d RobotModel::inertia_matrix(const Eigen::Vector3d& q) const {
    std::vector<DM> args = {to_dm(q)};
    auto res = inertia_fn_(args);
    return to_mat3(res.at(0));
}

Eigen::Matrix3d RobotModel::coriolis_matrix(
    const Eigen::Vector3d& q,
    const Eigen::Vector3d& dq) const {
    std::vector<DM> args = {to_dm(q), to_dm(dq)};
    auto res = coriolis_fn_(args);
    return to_mat3(res.at(0));
}

Eigen::Vector3d RobotModel::gravity_vector(const Eigen::Vector3d& q) const {
    std::vector<DM> args = {to_dm(q)};
    auto res = gravity_fn_(args);
    return to_vec3(res.at(0));
}

std::array<Eigen::Matrix4d, 6> RobotModel::forward_kinematics(
    const Eigen::Vector3d& q) const {
    std::vector<DM> args = {to_dm(q)};
    auto res = fk_fn_(args);
    std::array<Eigen::Matrix4d, 6> out;
    for (int i = 0; i < 6; ++i) {
        // fk_fn_ returns each frame as reshape(T, 16, 1): a column-major
        // layout of the 4x4 transform, so Eigen::Map<Matrix4d> reconstructs
        // the original 4x4.
        out[i] = to_mat4(res.at(i));
    }
    return out;
}

}  // namespace phantom_model
