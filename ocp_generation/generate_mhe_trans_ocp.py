"""
Generate acados C code for a TRANSLATIONAL-ONLY MHE (mass + disturbance).

Key idea (avoids the quaternion-state fragility of the full MHE): attitude is
MEASURED, not estimated. The thrust direction a = R·e3 (from the measured
quaternion) and the thrust magnitude T enter as KNOWN INPUTS. The estimator only
integrates the translational dynamics, so there is no quaternion state to drift
over the 1.0 s window → robust regardless of yaw/attitude.

State   x ∈ ℝ¹⁰ = [p(3), v(3), m, d(3)]
Noise   w ∈ ℝ⁴  = [w_m, w_d(3)]   (MHE "controls")
Inputs (params, known): T (thrust), a = R·e3 (world thrust direction, unit)

Dynamics:
  ṗ = v
  v̇ = -g·e₃ + (T/m)·a + d
  ṁ = w_m,  ḋ = w_d

Runtime params p ∈ ℝ³⁰:
  p[0:6]   = y_k  measurement [p(3), v(3)]
  p[6]     = T
  p[7:10]  = a = R·e3
  p[10:20] = x̄  prior (10D)
  p[20:30] = P̄_inv arrival weights (10D)

N=31, dt=1.5/31≈0.0484 (1.5 s window — matches NMPC horizon). Output: ../c_generated_code_mhe_trans/
"""
import os, shutil
import numpy as np
from casadi import MX, vertcat, norm_2, sqrt, dot
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
from quad_config import G

NX = 10   # [p,v,m,d]
NU = 4    # [w_m, w_d(3)]
NP = 34   # [y(6), T(1), a(3), x̄(10), P̄_inv(10), c_drag(1), sf_meas(3)]
N  = 31
DT = 0.01   # 10 ms grid = control period → pure 100 Hz MHE (310 ms window, for filtering)

R_P = 50.0                       # position measurement noise inverse
R_VXY, R_VZ = 10.0, 10.0   # ISOTROPIC velocity measurement weight
# STEP 1 (momentum observer): the IMU accelerometer measures the specific force
# sf = R·a_imu = (f/m)·a + d - drag (verified empirically). This residual estimates
# the disturbance d directly → the MHE becomes a VIRTUAL FORCE SENSOR.
W_IMU = 2.0                # IMU specific-force residual weight (accel is noisy → moderate)
# d UNLOCKED so it absorbs the accelerometer-sensed external force (momentum observer).
# Env-overridable to sweep the d "speed" (locked / slow / fast) vs mass estimate.
QW_M    = 1.0/0.001
QW_D    = float(os.environ.get("MHE_QW_D",  1.0e3))   # penalty on d motion (high=slow/locked)
W_D_MAX = float(os.environ.get("MHE_WD_MAX", 0.5))    # rate bound on d (low=slow)
D_MAX   = float(os.environ.get("MHE_D_MAX",  2.0))    # d magnitude bound
W_M_MAX = 0.1
M_MIN, M_MAX = 0.5, 3.0
D_MIN = -D_MAX


def build_model():
    model = AcadosModel()
    model.name = "quadrotor_mhe_trans"
    e3 = MX([0, 0, 1])

    p_s = MX.sym("p", 3)
    v_s = MX.sym("v", 3)
    m_s = MX.sym("m")
    d_s = MX.sym("d", 3)
    x = vertcat(p_s, v_s, m_s, d_s)

    w_m = MX.sym("w_m")
    w_d = MX.sym("w_d", 3)
    u_noise = vertcat(w_m, w_d)

    p_sym = MX.sym("p_rt", NP)
    y_p = p_sym[0:3]
    y_v = p_sym[3:6]
    T   = p_sym[6]
    a   = p_sym[7:10]   # R·e3 (world thrust direction)
    c_drag = p_sym[30]  # quadratic drag accel coeff (fixed, identified offline)

    dp = v_s
    # Drag is a KNOWN function of the measured velocity (v² signature) → distinct
    # from the thrust/mass term → does NOT confound m the way a free d would.
    # Safe norm (eps inside sqrt) → finite Jacobian at v=0.
    vmag = sqrt(dot(v_s, v_s) + 1e-6)
    dv = -G*e3 + (T / m_s) * a + d_s - c_drag * v_s * vmag
    dm = w_m
    dd = w_d
    f_expl = vertcat(dp, dv, dm, dd)

    x_dot = MX.sym("x_dot", NX)
    model.f_impl_expr = x_dot - f_expl
    model.f_expl_expr = f_expl
    model.x, model.xdot, model.u, model.p = x, x_dot, u_noise, p_sym
    return model, p_sym, y_p, y_v, p_s, v_s, m_s, d_s


def build_ocp():
    ocp = AcadosOcp()
    model, p_sym, y_p, y_v, p_s, v_s, m_s, d_s = build_model()
    ocp.model = model
    ocp.code_export_directory = os.path.join(SCRIPT_DIR, "..", "c_generated_code_mhe_trans")
    ocp.solver_options.N_horizon = N
    u_noise = model.u

    R_inv  = np.diag([R_P]*3 + [R_VXY, R_VXY, R_VZ])   # 6x6 — lean on v_z for mass
    QW_inv = np.diag([QW_M, QW_D, QW_D, QW_D])  # 4x4

    e_meas = vertcat(p_s - y_p, v_s - y_v)      # 6D — all Euclidean, NO quaternion
    ocp.cost.cost_type = "EXTERNAL"

    # IMU specific-force residual (momentum observer): the accelerometer measures
    # sf_meas = R·a_imu, which the model predicts as (T/m)·a + d - drag. Penalizing
    # their mismatch drives the disturbance d → a virtual force sensor.
    sf_meas = p_sym[31:34]
    T_p, a_p, cdrag_p = p_sym[6], p_sym[7:10], p_sym[30]
    vmag_c = sqrt(dot(v_s, v_s) + 1e-6)
    sf_model = (T_p / m_s) * a_p + d_s - cdrag_p * v_s * vmag_c
    r_imu = sf_meas - sf_model

    ocp.model.cost_expr_ext_cost = (e_meas.T @ R_inv @ e_meas
                                    + u_noise.T @ QW_inv @ u_noise
                                    + W_IMU * dot(r_imu, r_imu))

    x_bar = p_sym[10:20]
    Pinv  = p_sym[20:30]
    e_arr = vertcat(p_s - x_bar[0:3], v_s - x_bar[3:6],
                    m_s - x_bar[6], d_s - x_bar[7:10])   # 10D Euclidean
    ocp.cost.cost_type_0 = "EXTERNAL"
    arrival = Pinv[0]*e_arr[0]**2
    for i in range(1, 10):
        arrival = arrival + Pinv[i]*e_arr[i]**2
    ocp.model.cost_expr_ext_cost_0 = arrival

    ocp.cost.cost_type_e = "EXTERNAL"
    ocp.model.cost_expr_ext_cost_e = (e_meas.T @ R_inv @ e_meas
                                      + W_IMU * dot(r_imu, r_imu))

    p_default = np.zeros(NP)
    p_default[6] = 9.81 * 1.0      # T nominal
    p_default[9] = 1.0             # a = e3 (level)
    p_default[16] = 1.0            # x̄_m nominal
    p_default[20:30] = 1.0         # P̄_inv unit
    p_default[30] = 0.0            # c_drag default 0 (set at runtime)
    p_default[33] = 9.81           # sf_meas nominal = g·e3 (hover specific force)
    ocp.parameter_values = p_default

    # bounds on m (idx 6), d (idx 7,8,9)
    lbx = np.array([M_MIN, D_MIN, D_MIN, D_MIN])
    ubx = np.array([M_MAX, D_MAX, D_MAX, D_MAX])
    idxbx = np.array([6, 7, 8, 9])
    ocp.constraints.lbx,   ocp.constraints.ubx,   ocp.constraints.idxbx   = lbx, ubx, idxbx
    ocp.constraints.lbx_e, ocp.constraints.ubx_e, ocp.constraints.idxbx_e = lbx, ubx, idxbx
    ocp.constraints.lbx_0, ocp.constraints.ubx_0, ocp.constraints.idxbx_0 = lbx, ubx, idxbx

    ocp.constraints.lbu = np.array([-W_M_MAX, -W_D_MAX, -W_D_MAX, -W_D_MAX])
    ocp.constraints.ubu = np.array([ W_M_MAX,  W_D_MAX,  W_D_MAX,  W_D_MAX])
    ocp.constraints.idxbu = np.array([0, 1, 2, 3])

    ocp.solver_options.qp_solver            = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.qp_solver_cond_N     = N // 4
    ocp.solver_options.qp_solver_iter_max   = 50
    ocp.solver_options.qp_solver_warm_start = 2
    ocp.solver_options.hessian_approx       = "GAUSS_NEWTON"
    ocp.solver_options.regularize_method    = "PROJECT_REDUC_HESS"
    ocp.solver_options.levenberg_marquardt  = 1e-2
    ocp.solver_options.hpipm_mode           = "ROBUST"
    ocp.solver_options.integrator_type      = "ERK"
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.sim_method_num_steps  = 1
    ocp.solver_options.nlp_solver_type      = "SQP_RTI"
    ocp.solver_options.tol                  = 1e-3
    ocp.solver_options.tf                   = N * DT
    return ocp


def main():
    ocp = build_ocp()
    code_dir = ocp.code_export_directory
    if os.path.isdir(code_dir):
        shutil.rmtree(code_dir)
    json_file = os.path.join(os.path.dirname(code_dir), f"acados_ocp_{ocp.model.name}.json")
    if os.path.isfile(json_file):
        os.remove(json_file)
    print(f"Generating TRANS-MHE (10D, no quaternion) in {code_dir} ...")
    AcadosOcpSolver(ocp, json_file=json_file)
    print(f"JSON: {json_file}\nNow: cd ../build && cmake .. && make")


if __name__ == "__main__":
    main()
