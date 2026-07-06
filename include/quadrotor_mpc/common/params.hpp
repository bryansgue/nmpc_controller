#pragma once

namespace quadrotor_mpc {

struct QuadParams {
    // Physical
    double mass    = 1.05;      // [kg] REAL mass = sum of MuJoCo geom masses (core .85 + arms .14 + thrusters .048 + props .012)
    double g       = 9.81;      // [m/s²]
    double tau_rc  = 0.03;      // [s] first-order rate lag

    // Thrust limits
    double T_max = 5.0 * 9.81;  // ≈49.05 N
    double T_min = 0.0;

    // Rate command limits
    double W_max = 20.0;         // [rad/s] max body rate command

    // Attitude reference
    double att_ref_max_tilt_deg = 60.0; // [deg]
    double att_ref_speed        = 15.0; // [m/s] nominal speed for attitude ref
};

struct SimParams {
    double dt      = 0.01;    // [s] control period (100 Hz)
    double t_final = 85.0;    // [s] max simulation time budget
};

struct NmpcParams {
    double dt           = 0.01;   // [s] control loop period (100 Hz)
    double t_prediction = 1.5;    // [s] prediction horizon

    double node_dt(int horizon_nodes) const {
        return t_prediction / static_cast<double>(horizon_nodes);
    }

    // Cost weights (default, can be overridden at runtime)
    Vec3 Q_pos   = Vec3(150.0, 150.0, 150.0);
    Vec3 Q_att   = Vec3(50.0, 50.0, 50.0);
    Eigen::Vector4d R_u = Eigen::Vector4d(0.1, 0.5, 0.5, 0.5);
};


}  // namespace quadrotor_mpc
