#pragma once
/**
 * MheTransController — translational-only MHE (mass + disturbance).
 *
 * State x ∈ ℝ¹⁰ = [p(3), v(3), m, d(3)]   (NO quaternion, NO ω → robust)
 * Process noise w ∈ ℝ⁴ = [w_m, w_d(3)]
 * Known inputs (params): T (thrust), a = R·e3 (world thrust direction, from the
 *   MEASURED quaternion). Attitude is measured, not estimated.
 *
 * Runtime params p ∈ ℝ³⁰:
 *   p[0:6]   = y_k  [p(3), v(3)]
 *   p[6]     = T
 *   p[7:10]  = a
 *   p[10:20] = x̄  prior (10D)
 *   p[20:30] = P̄_inv (10D, Euclidean)
 */
#include "quadrotor_mpc/common/types.hpp"
#include <Eigen/Dense>
#include <deque>

struct quadrotor_mhe_trans_solver_capsule;

namespace quadrotor_mpc {

using State10 = Eigen::Matrix<double, 10, 1>;

struct MheTransEstimate {
    Vec3   pos;   Vec3 vel;
    double m_hat = 1.05;
    Vec3   d_hat = Vec3::Zero();

    State10 to_vector() const {
        State10 v; v.head<3>() = pos; v.segment<3>(3) = vel;
        v(6) = m_hat; v.tail<3>() = d_hat; return v;
    }
    static MheTransEstimate from_vector(const State10& v) {
        MheTransEstimate e; e.pos = v.head<3>(); e.vel = v.segment<3>(3);
        e.m_hat = v(6); e.d_hat = v.tail<3>(); return e;
    }
};

class MheTransController {
public:
    static const int N  = 31;
    static const int NX = 10;
    static const int NU = 4;
    static const int NP = 34;

    /// Fixed quadratic-drag acceleration coefficient (a_drag = -c·v·|v|).
    void set_drag(double c) { drag_c_ = c; }

    MheTransController();
    ~MheTransController();
    bool init();

    void reset(const MheTransEstimate& x0_prior,
               const Eigen::Matrix<double,10,1>& P_bar_diag);

    /// y = [p(3), v(3)] (6D); inputs T (thrust) and a = R·e3 (world dir).
    void push(const Eigen::Matrix<double,6,1>& y_k, double T, const Vec3& a,
              const Vec3& sf_meas = Vec3(0, 0, 9.81));

    int solve();
    MheTransEstimate get_estimate() const;
    double get_sigma() const { return sigma_k_; }
    double get_solve_time() const { return solve_time_s_; }
    void propagate_prior(bool solver_ok = true);
    void free();

private:
    quadrotor_mhe_trans_solver_capsule* capsule_ = nullptr;
    std::deque<Eigen::Matrix<double,6,1>> y_win_;
    std::deque<double>                    T_win_;
    std::deque<Vec3>                      a_win_;
    std::deque<Vec3>                      sf_win_;   // IMU specific force R·a_imu

    State10                    x_bar_;
    Eigen::Matrix<double,10,1> P_bar_inv_;
    Eigen::Matrix<double,10,1> P_bar_diag_;

    double sigma_k_      = 1e6;
    double solve_time_s_ = 0.0;
    double drag_c_       = 0.0;   // quadratic drag coeff
    static constexpr double Q_W_M = 1e-4;
    static constexpr double Q_W_D = 1e-3;
    static constexpr double DT    = 0.01;   // 10 ms grid = control period → pure 100 Hz
    static constexpr double P_MIN_PHYS  = 1e-4;
    static constexpr double P_MIN_PARAM = 1e-3;

    void set_stage_params_(int k);
};

}  // namespace quadrotor_mpc
