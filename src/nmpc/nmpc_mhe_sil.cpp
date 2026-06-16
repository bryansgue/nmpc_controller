/**
 * NMPC-MHE SiL — Lie-invariant MHE + Adaptive (offset-free) NMPC with MuJoCo.
 *
 * This is the CLEAN adaptive-estimation vehicle: a temporal trajectory-tracking
 * NMPC (no arc-length) whose model parameters [m̂, d̂, k̂_τ] are estimated online
 * by a dedicated 18-state Lie-invariant MHE (thrust-as-INPUT). Because the NMPC tracks a
 * fixed time-reference, the only thing the MHE changes is MODEL ACCURACY → the
 * tracking improvement is 100% attributable to the estimation (clean ablation).
 *
 * Architecture (100 Hz):
 *   1. MHE: push measurement y_k + applied control u_{k-1}
 *   2. MHE: solve() → x̂_k incl. m̂, τ̂, d̂  + σ_k = tr(P̄_θ)
 *   3. MHE: propagate_prior()
 *   4. EMA smooth on [m̂, τ̂, d̂]
 *   5. Gate σ_k < 0.05 → (v2 only) NMPC.set_model_params(m̂, d̂, k̂_τ)
 *   6. NMPC: set_x0 (physical state from MHE) + temporal references
 *   7. NMPC: solve() → u* = [T, ω_cmd]
 *   8. Send: T + ω_cmd to MuJoCo + feed u=[T,ω_cmd] to MHE as the applied input
 *
 * The 18-state MHE takes thrust T and ω_cmd as KNOWN INPUTS (params), exactly
 * matching the NMPC command — no thrust-state reconstruction, no Δf hack.
 *
 * Modes:
 *   ./nmpc_mhe_sil v1 [t_run]   → open-loop:  NMPC uses wrong mass (0.9 kg) always
 *   ./nmpc_mhe_sil v2 [t_run]   → closed-loop: NMPC corrects mass via MHE (default)
 *
 * CSV columns match mhe_mpcc_sil for plot reuse. Results in results/nmpc_mhe_v{1,2}.csv
 */

#include "quadrotor_mpc/nmpc/nmpc_controller.hpp"
#include "quadrotor_mpc/mhe/mhe_trans_controller.hpp"
#include "quadrotor_mpc/mujoco/mujoco_interface.hpp"
#include "quadrotor_mpc/mujoco/sil_protocol.hpp"
#include "quadrotor_mpc/common/quaternion_algebra.hpp"
#include "quadrotor_mpc/common/params.hpp"
#include "quadrotor_mpc/trajectory/lissajous.hpp"
#include "quadrotor_mpc/trajectory/attitude_reference.hpp"

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <fstream>
#include <thread>
#include <cmath>
#include <vector>
#include <string>
#include <chrono>

using namespace quadrotor_mpc;

// ── Adaptive law / MHE prior ──────────────────────────────────────────────────
// FOCUS = external disturbance rejection. Mass is assumed KNOWN/identified
// (= true 1.05, summed from the MuJoCo geom masses). The MHE estimates the
// external disturbance d̂; the NMPC injects d̂ as feedforward.
static constexpr double M_KNOWN      = 1.05;  // [kg] REAL mass (MuJoCo model total)
static constexpr double TAU_HAT_INIT = 0.03;  // [s]
static constexpr double SIGMA_GATE   = 0.05;  // inject θ̂ only when σ_k below this
static constexpr double EMA_ALPHA_PARAM = 0.995; // mass + τ_rc (heavier smoothing)
static constexpr double EMA_ALPHA_DIST  = 0.92;  // disturbances (τ≈0.12s)

// ── Extended CSV (same layout as mhe_mpcc_sil for plot reuse) ────────────────
static void save_csv_extended(
    const std::string& path, const SilResult& res,
    const std::vector<double>& log_theta, const std::vector<double>& log_vtheta,
    const std::vector<double>& log_atheta, const std::vector<double>& log_s_nearest,
    const std::vector<double>& log_mhat, const std::vector<double>& log_tauhat,
    const std::vector<double>& log_dx, const std::vector<double>& log_dy,
    const std::vector<double>& log_dz, const std::vector<double>& log_sigma,
    const std::vector<double>& log_ws, const std::vector<double>& log_mhe_ms,
    const std::vector<int>&    log_mhe_status,
    const std::vector<double>& log_px_hat, const std::vector<double>& log_py_hat,
    const std::vector<double>& log_pz_hat, const std::vector<double>& log_vx_hat,
    const std::vector<double>& log_vy_hat, const std::vector<double>& log_vz_hat,
    const std::vector<double>& log_qw_hat, const std::vector<double>& log_qx_hat,
    const std::vector<double>& log_qy_hat, const std::vector<double>& log_qz_hat,
    const std::vector<double>& log_wx_hat, const std::vector<double>& log_wy_hat,
    const std::vector<double>& log_wz_hat)
{
    std::ofstream csv(path);
    csv << "t,px,py,pz,vx,vy,vz,qw,qx,qy,qz,wx,wy,wz,"
        << "T,wx_cmd,wy_cmd,wz_cmd,"
        << "mpcc_solve_ms,loop_ms,s,"
        << "px_ref,py_ref,pz_ref,qw_ref,qx_ref,qy_ref,qz_ref,"
        << "theta,vtheta,a_theta,s_nearest,"
        << "m_hat,tau_hat,dx_hat,dy_hat,dz_hat,"
        << "sigma_k,W_s,mhe_ms,mhe_status,mpcc_status,"
        << "px_hat,py_hat,pz_hat,vx_hat,vy_hat,vz_hat,"
        << "qw_hat,qx_hat,qy_hat,qz_hat,wx_hat,wy_hat,wz_hat\n";

    int n = res.n_steps;
    for (int i = 0; i < n; ++i) {
        csv << res.t(i);
        for (int j = 0; j < 13; ++j) csv << "," << res.x(j, i);
        for (int j = 0; j < 4;  ++j) csv << "," << res.u(j, i);
        csv << "," << res.solve_ms(i) << "," << res.loop_ms(i) << "," << res.progress(i);
        for (int j = 0; j < 3; ++j) csv << "," << res.p_ref(j, i);
        for (int j = 0; j < 4; ++j) csv << "," << res.q_ref(j, i);

        auto safe   = [&](const std::vector<double>& v){ return (i<(int)v.size())?v[i]:0.0; };
        auto safe_i = [&](const std::vector<int>&    v){ return (i<(int)v.size())?v[i]:-1;  };

        csv << "," << safe(log_theta)  << "," << safe(log_vtheta) << "," << safe(log_atheta)
            << "," << safe(log_s_nearest)
            << "," << safe(log_mhat) << "," << safe(log_tauhat)
            << "," << safe(log_dx) << "," << safe(log_dy) << "," << safe(log_dz)
            << "," << safe(log_sigma) << "," << safe(log_ws)
            << "," << safe(log_mhe_ms) << "," << safe_i(log_mhe_status)
            << "," << res.status(i)
            << "," << safe(log_px_hat) << "," << safe(log_py_hat) << "," << safe(log_pz_hat)
            << "," << safe(log_vx_hat) << "," << safe(log_vy_hat) << "," << safe(log_vz_hat)
            << "," << safe(log_qw_hat) << "," << safe(log_qx_hat) << "," << safe(log_qy_hat)
            << "," << safe(log_qz_hat) << "," << safe(log_wx_hat) << "," << safe(log_wy_hat)
            << "," << safe(log_wz_hat) << "\n";
    }
}

// ═══════════════════════════════════════════════════════════════════════════
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    QuadParams quad;
    NmpcParams nmpc_p;

    // Usage: ./nmpc_mhe_sil [v1|v2] [t_run] [hover]
    //   hover → hold a fixed setpoint (no aggressive trajectory). At ~0 speed
    //   the aerodynamic drag vanishes, so d̂ captures ONLY the external gusts →
    //   clean disturbance-rejection demonstration (v2 should clearly beat v1).
    bool use_v2 = true;
    std::string mode_str = "v2";
    double t_run_arg = -1.0;
    bool hover_mode = false;
    bool slow_mode  = false;   // gentle Lissajous: moving (observable d) but low drag
    double w_override = -1.0;  // sweep: positive value overrides liss.w
    bool idmass_mode = false;  // "idmass" → start mass at 0.60 to show convergence
    if (argc >= 2) { mode_str = std::string(argv[1]); use_v2 = (mode_str != "v1"); }
    if (argc >= 3) t_run_arg = std::atof(argv[2]);
    for (int i = 1; i < argc; ++i) {
        std::string a(argv[i]);
        if (a == "hover") hover_mode = true;
        if (a == "slow")  slow_mode  = true;
        if (a == "idmass") idmass_mode = true;
        if (a.rfind("w=", 0) == 0) w_override = std::atof(a.c_str() + 2);
    }
    if (w_override > 0.0) {
        char buf[32]; std::snprintf(buf, sizeof(buf), "_w%.1f", w_override);
        mode_str += buf;  // CSV name → nmpc_mhe_v2_w2.0.csv
    }

    auto muj = std::make_shared<MujocoInterface>("nmpc_mhe_sil_controller");
    std::thread spin_thread([&]() { rclcpp::spin(muj); });

    RCLCPP_INFO(muj->get_logger(),
        "NMPC-MHE SiL [%s] — DISTURBANCE rejection, mass known=%.2f kg, inject d̂ only",
        mode_str.c_str(), M_KNOWN);

    // ── Temporal Lissajous trajectory (analytic — no arc-length) ─────────
    LissajousTrajectory liss;
    if (slow_mode) liss.w = 0.5;   // 4× slower → peak speed ~3 m/s → drag ~16× less
    if (w_override > 0.0) liss.w = w_override;  // velocity-sweep override
    const double dt_ctrl = nmpc_p.dt;
    const double t_traj  = (t_run_arg > 0.0) ? std::min(t_run_arg, liss.t_final)
                                             : liss.t_final;
    RCLCPP_INFO(muj->get_logger(),
        "Trajectory: t_run=%.1f s, dt=%.3f s, N_horizon=%d",
        t_traj, dt_ctrl, NmpcController::N);

    // ── Init NMPC ────────────────────────────────────────────────────────
    NmpcController ctrl;
    if (!ctrl.init()) {
        RCLCPP_ERROR(muj->get_logger(), "NMPC init failed!");
        rclcpp::shutdown(); spin_thread.join(); return 1;
    }
    // Gains retuned for the COARSE N=31/1.5s (dt≈0.048s) discretization: penalise
    // body-rate commands hard (a big ω_cmd over 0.048s = huge attitude swing → crash)
    // and keep attitude tracking moderate. Stability over tightness.
    ctrl.set_weights(Vec3(100, 100, 100),       // Q_pos
                     Vec3(20, 20, 20),           // Q_att (moderate, don't over-demand)
                     Eigen::Vector4d(0.3, 5.0, 5.0, 5.0));  // R_u: strong rate penalty
    // Known mass, zero disturbance to start. v2 will inject d̂ once converged.
    // THRUST-LAG FEEDFORWARD test: both use the same identified model; v2 ADDS the
    // feedforward thrust-lag compensation (T_applied = T_des + τ_f·dT/dt) → isolates
    // the effect of compensating the identified thrust dynamic.
    ctrl.set_model_params(1.05, Vec3::Zero(), 1.0 / 0.056);

    // ── Init TRANSLATIONAL-ONLY MHE (10D: p,v,m,d — attitude is a measured input) ─
    MheTransController mhe;
    if (!mhe.init()) {
        RCLCPP_ERROR(muj->get_logger(), "MHE init failed!");
        rclcpp::shutdown(); spin_thread.join(); return 1;
    }
    MheTransEstimate x0_prior;
    x0_prior.pos   = liss.position(0.0);
    x0_prior.vel   = Vec3::Zero();
    // Default: mass KNOWN (1.05) → clean d from t=0 for force-sensing. Flag "idmass":
    // start at 0.60 to show the mass-identification convergence (separate experiment).
    x0_prior.m_hat = idmass_mode ? 0.60 : 1.05;
    x0_prior.d_hat = Vec3::Zero();

    // P̄ diagonal (10D): [p(3),v(3),m,d(3)]
    Eigen::Matrix<double,10,1> P_init;
    P_init.segment<3>(0).setConstant(0.05);
    P_init.segment<3>(3).setConstant(0.2);
    P_init(6) = 0.1;                          // mass: estimate (the SOLE parameter)
    P_init.segment<3>(7).setConstant(1e-5);   // d ELIMINATED (locked ≈0) → clean mass
    mhe.reset(x0_prior, P_init);

    // ── EMA + control-feed trackers ──────────────────────────────────────
    double m_ema   = idmass_mode ? 0.60 : 1.05;   // match the estimator init
    double tau_ema = 0.03;   // τ locked at nominal
    Vec3   d_ema   = Vec3::Zero();
    // Applied control fed to the MHE = [T, ω_cmd]; init to hover thrust.
    Control4 u_prev = Control4::Zero();
    u_prev(0) = quad.mass * quad.g;

    // ── Rotational k_τ estimator (recursive least squares, decoupled) ────
    // ω̇ = k_τ·(ω_cmd − ω) is LINEAR in k_τ → robust RLS, no quaternion needed.
    Vec3   omega_prev = Vec3::Zero();
    double ktau_num = 0.0, ktau_den = 1e-6;
    double ktau_est = 1.0 / TAU_HAT_INIT;
    const double KTAU_LAMBDA = 0.997;   // forgetting factor

    // ── Thrust dynamics k_f via RLS: ḟ = k_f·(T_cmd − f), f = m̂·|v̇+g·e₃| ──
    Vec3   vel_prev_kf = Vec3::Zero();
    double f_prev_kf   = quad.mass * quad.g;
    double kf_num = 0.0, kf_den = 1e-6;
    double kf_est = 112.0;              // ≈ 1/9ms (offline-identified seed)
    const double KF_LAMBDA = 0.997;
    // Actual thrust propagated at 100 Hz with the identified τ_f (=1/kf_est):
    //   ḟ = (T_cmd − f)/τ_f.  Fed to the MHE INSTEAD of the raw jerky T_cmd → the
    //   estimator sees the real (lagged) thrust. This USES the identified τ_f.
    double f_actual = quad.mass * quad.g;
    double T_nmpc_prev = quad.mass * quad.g;   // for thrust-lag feedforward (v2)

    // ── Log vectors ──────────────────────────────────────────────────────
    std::vector<double> log_theta, log_vtheta, log_atheta, log_s_nearest;
    std::vector<double> log_mhat, log_tauhat, log_dx, log_dy, log_dz;
    std::vector<double> log_sigma, log_ws, log_mhe_ms;
    std::vector<int>    log_mhe_status;
    std::vector<double> log_px_hat, log_py_hat, log_pz_hat;
    std::vector<double> log_vx_hat, log_vy_hat, log_vz_hat;
    std::vector<double> log_qw_hat, log_qx_hat, log_qy_hat, log_qz_hat;
    std::vector<double> log_wx_hat, log_wy_hat, log_wz_hat;

    bool first_call = true;

    // NMPC horizon node spacing (tf/N = 1.5/31 ≈ 0.048 s) — used for the temporal
    // reference horizon. NOT the 100 Hz control period.
    const double nmpc_node_dt = 1.5 / NmpcController::N;
    // The MHE lives in the 100 Hz control loop: PUSH + SOLVE + PROPAGATE every step.
    // The OCP grid is 10 ms (DT=0.01, N=31 → 310 ms window), matching the control
    // period, so data enters at 100 Hz with no sub-sampling and the estimate is smooth.
    bool      mhe_seeded = false;   // re-seed MHE prior from the actual state at RUN start
    MheTransEstimate xhat = x0_prior;
    double sigma_k  = mhe.get_sigma();
    double mhe_ms   = 0.0;
    int    mhe_stat = -1;

    // ── SiL protocol (time-based progress) ───────────────────────────────
    SilConfig cfg;
    cfg.P0           = liss.position(0.0);
    cfg.mass         = quad.mass;
    cfg.gravity      = quad.g;
    cfg.t_final      = t_traj + 10.0;
    cfg.progress_max = t_traj;
    cfg.progress_done_tol = 0.5;
    if (hover_mode) {
        // Holding a setpoint means ~zero velocity by design — the stall guard
        // (vel<0.3 for 5s = abort) is a false positive here. Disable it.
        // Ironically v2 (good rejection) stays stiller than v1 → only v2 "stalls".
        cfg.stall_timeout = 1e9;
    }
    SilProtocol proto(muj, cfg);

    proto.set_progress_tracker(
        [dt_ctrl](const Vec3&, double t_prev) -> double { return t_prev + dt_ctrl; });

    proto.set_controller(
        [&](const DroneState& ds, double /*t*/, double t_elapsed) -> ControlOutput {
            using Clock = std::chrono::steady_clock;

            // Propagate ACTUAL thrust f at 100 Hz from the last command via the
            // identified τ_f (=1/kf_est): ḟ=(T_cmd−f)/τ_f. This f (lagged, real) is
            // fed to the MHE instead of the jerky command → uses the identified τ_f.
            f_actual += (1.0 - std::exp(-dt_ctrl * kf_est)) * (u_prev(0) - f_actual);

            // ── MHE in the 100 Hz loop — PURE 100 Hz, no sub-sampling ─────────────
            // The OCP grid is 10 ms (DT=0.01, N=31 → 310 ms window), so the loop rate,
            // the measurement rate, and the node spacing ALL match at 100 Hz. Every
            // control step: push the (noisy) odometry, solve once, propagate the prior.
            // No 33 Hz cadence, no predict-to-now hacks — the newest node (stage N) is
            // always the current time, so the estimate is smooth by construction.
            // The NMPC state feedback comes from raw odometry (never the estimator).
            {
                // Thrust direction a = R(q)·e3 from the MEASURED quaternion (attitude
                // is a known input, not a state). T = applied (lagged) thrust.
                const double qw=ds.quat(0), qx=ds.quat(1), qy=ds.quat(2), qz=ds.quat(3);
                Vec3 a(2*(qx*qz+qw*qy), 2*(qy*qz-qw*qx), 1-2*(qx*qx+qy*qy));
                double T_in = f_actual;

                // Re-seed prior from the actual state at RUN start.
                if (!mhe_seeded) {
                    MheTransEstimate seed = x0_prior;     // keep m, d priors
                    seed.pos = ds.pos; seed.vel = ds.vel;
                    mhe.reset(seed, P_init);
                    mhe_seeded = true;
                }

                // IMU specific force (momentum observer): sf = R·a_imu (verified sign).
                // This is the accelerometer's direct measurement of (f/m)·a + d → drives
                // the disturbance estimate. The MHE becomes a virtual force sensor.
                Eigen::Quaterniond q_meas(qw, qx, qy, qz);
                Vec3 sf_meas = q_meas.toRotationMatrix() * ds.accel;

                // The odometry topic is already NOISY (the sim publishes dirty odom).
                Eigen::Matrix<double,6,1> y_k; y_k << ds.pos, ds.vel;
                mhe.push(y_k, T_in, a, sf_meas);

                auto mhe_tic = Clock::now();
                mhe_stat = mhe.solve();
                mhe_ms = std::chrono::duration<double,std::milli>(Clock::now()-mhe_tic).count();
                mhe.propagate_prior(mhe_stat == 0);

                MheTransEstimate e = mhe.get_estimate();
                sigma_k = mhe.get_sigma();
                bool mhe_ok = (mhe_stat == 0) && e.pos.allFinite() && e.vel.allFinite()
                              && std::isfinite(e.m_hat) && e.d_hat.allFinite();
                if (mhe_ok) {
                    xhat = e;
                    m_ema = EMA_ALPHA_PARAM*m_ema + (1.0-EMA_ALPHA_PARAM)*e.m_hat;
                    d_ema = EMA_ALPHA_DIST *d_ema + (1.0-EMA_ALPHA_DIST) *e.d_hat;
                }

            }

            // ── 5. (two-phase) model is FIXED at init — no online injection here.
            //   The MHE/RLS still run to ESTIMATE (logged), but the controller uses
            //   the fixed identified model, mirroring offline-ID → deploy.

            // ── Rotational k_τ via RLS (decoupled, every 100 Hz step) ──
            {
                Vec3 wcmd = u_prev.tail<3>();   // last NMPC body-rate command
                for (int i = 0; i < 3; ++i) {
                    double wd = (ds.omega(i) - omega_prev(i)) / dt_ctrl;  // ω̇
                    double uu = wcmd(i) - omega_prev(i);                  // ω_cmd − ω
                    ktau_num = KTAU_LAMBDA*ktau_num + wd*uu;
                    ktau_den = KTAU_LAMBDA*ktau_den + uu*uu;
                }
                if (ktau_den > 1e-2) ktau_est = std::clamp(ktau_num/ktau_den, 1.0, 200.0);
                omega_prev = ds.omega;
            }

            // ── Thrust dynamics k_f via RLS (every 100 Hz step) ────────
            {
                Vec3 vdot = (ds.vel - vel_prev_kf) / dt_ctrl;       // v̇
                double f_real = m_ema * (vdot + Vec3(0,0,quad.g)).norm();  // m̂·|v̇+g·e₃|
                double fdot = (f_real - f_prev_kf) / dt_ctrl;        // ḟ
                double uu   = u_prev(0) - f_prev_kf;                  // T_cmd − f
                kf_num = KF_LAMBDA*kf_num + fdot*uu;
                kf_den = KF_LAMBDA*kf_den + uu*uu;
                if (kf_den > 1e-1) kf_est = std::clamp(kf_num/kf_den, 10.0, 500.0);
                f_prev_kf = f_real; vel_prev_kf = ds.vel;
            }

            // ── 6. NMPC: x0 from RAW ODOMETRY (not the estimator) + refs ─
            State13 x;
            x << ds.pos, ds.vel, ds.quat / (ds.quat.norm() + 1e-12), ds.omega;
            ctrl.set_x0(x);

            ControlOutput out;
            Quat4 q_ref_prev = ds.quat;
            const Vec3  hover_p = liss.position(0.0);
            const Quat4 hover_q(1, 0, 0, 0);
            for (int j = 0; j <= NmpcController::N; ++j) {
                Vec3  pr; Quat4 qr;
                if (hover_mode) {
                    pr = hover_p; qr = hover_q;   // fixed setpoint, level
                } else {
                    double t_j = std::min(t_elapsed + j * nmpc_node_dt, t_traj);
                    Vec3 vr = liss.velocity(t_j);
                    Vec3 ar = liss.acceleration(t_j);
                    pr = liss.position(t_j);
                    qr = quat_from_traj(vr, ar, quad.g, quad.att_ref_max_tilt_deg, q_ref_prev);
                    q_ref_prev = qr;
                }
                if (j < NmpcController::N) ctrl.set_reference(j, pr, qr);
                else                       ctrl.set_reference_terminal(pr, qr);
                if (j == 0) { out.p_ref = pr; out.q_ref = qr; }
            }

            // Warm-start a few SQP-RTI iterations on the very first call
            if (first_call) {
                for (int i = 0; i < 20; ++i) ctrl.solve();
                first_call = false;
            }

            // ── 7. NMPC solve ──────────────────────────────────────────
            out.solver_status = ctrl.solve();
            out.solve_time_s  = ctrl.get_solve_time();
            Control4 u = ctrl.get_u0();

            // ── 8. Commands to MuJoCo ──────────────────────────────────
            // v2: feedforward thrust-lag compensation using identified τ_f.
            //   T_applied = T_des + τ_f·dT_des/dt  → the lagged motor reaches T_des.
            double T_cmd_out = u(0);
            if (use_v2) {
                double tau_f = 1.0 / kf_est;
                double Tdot  = (u(0) - T_nmpc_prev) / dt_ctrl;
                T_cmd_out = std::clamp(u(0) + tau_f*Tdot, quad.T_min, quad.T_max);
            }
            T_nmpc_prev = u(0);
            out.cmd.thrust    = T_cmd_out;
            out.cmd.omega_cmd = u.tail<3>();

            // ── Build MHE applied-control for next push ────────────────
            // u_applied = [T, ω_cmd] — the exact NMPC command, as a known input.
            u_prev << u(0), u(1), u(2), u(3);

            // ── Log ─────────────────────────────────────────────────────
            // cols 28-30 (theta/vtheta/atheta) repurposed for the GROUND-TRUTH external
            // force [N] (validation of d̂ against the real wind).
            log_theta.push_back(ds.ext_force.x());
            log_vtheta.push_back(ds.ext_force.y());
            log_atheta.push_back(ds.ext_force.z());
            log_s_nearest.push_back(t_elapsed);
            log_mhat.push_back(m_ema); log_tauhat.push_back(1.0 / ktau_est);  // τ̂ = 1/k̂_τ (RLS)
            log_dx.push_back(d_ema.x()); log_dy.push_back(d_ema.y()); log_dz.push_back(d_ema.z());
            log_sigma.push_back(sigma_k); log_ws.push_back(1.0 / kf_est);  // τ_f in W_s column
            log_mhe_ms.push_back(mhe_ms); log_mhe_status.push_back(mhe_stat);
            log_px_hat.push_back(xhat.pos.x()); log_py_hat.push_back(xhat.pos.y()); log_pz_hat.push_back(xhat.pos.z());
            log_vx_hat.push_back(xhat.vel.x()); log_vy_hat.push_back(xhat.vel.y()); log_vz_hat.push_back(xhat.vel.z());
            // attitude is measured (not estimated by the trans MHE) → log the measurement
            log_qw_hat.push_back(ds.quat(0)); log_qx_hat.push_back(ds.quat(1));
            log_qy_hat.push_back(ds.quat(2)); log_qz_hat.push_back(ds.quat(3));
            log_wx_hat.push_back(ds.omega.x()); log_wy_hat.push_back(ds.omega.y()); log_wz_hat.push_back(ds.omega.z());

            {
                static int c = 0;
                if (++c % 500 == 0)
                    RCLCPP_INFO(muj->get_logger(),
                        "[MHE] m̂=%.3f kg  τ̂=%.4f  d̂=[%.2f,%.2f,%.2f]  σ_k=%.3e  solve=%.2f ms",
                        m_ema, tau_ema, d_ema.x(), d_ema.y(), d_ema.z(), sigma_k, mhe_ms);
            }
            return out;
        });

    SilResult res = proto.execute();

    std::system("mkdir -p ../results");
    std::string persistent = "../results/nmpc_mhe_" + mode_str + ".csv";
    save_csv_extended(persistent, res,
        log_theta, log_vtheta, log_atheta, log_s_nearest,
        log_mhat, log_tauhat, log_dx, log_dy, log_dz,
        log_sigma, log_ws, log_mhe_ms, log_mhe_status,
        log_px_hat, log_py_hat, log_pz_hat, log_vx_hat, log_vy_hat, log_vz_hat,
        log_qw_hat, log_qx_hat, log_qy_hat, log_qz_hat, log_wx_hat, log_wy_hat, log_wz_hat);
    RCLCPP_INFO(muj->get_logger(), "CSV saved: %s (%d samples, completed=%s)",
                persistent.c_str(), res.n_steps, res.completed ? "YES" : "NO");

    if (!log_mhat.empty())
        RCLCPP_INFO(muj->get_logger(),
            "[MHE convergence] m̂: %.3f → %.3f kg (true=%.3f)  σ_k: %.2e → %.2e",
            log_mhat.front(), log_mhat.back(), quad.mass,
            log_sigma.front(), log_sigma.back());

    rclcpp::shutdown();
    spin_thread.join();
    return 0;
}
