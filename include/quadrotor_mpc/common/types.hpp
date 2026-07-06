#pragma once

#include <Eigen/Dense>

namespace quadrotor_mpc {

// NMPC: 13 states [p(3), v(3), q(4), ω(3)], 4 controls [T, ωx, ωy, ωz]
using State13  = Eigen::Matrix<double, 13, 1>;
using Control4 = Eigen::Matrix<double, 4, 1>;

// Quaternion [qw, qx, qy, qz]
using Quat4 = Eigen::Vector4d;

// Rotation matrix
using Mat3 = Eigen::Matrix3d;
using Vec3 = Eigen::Vector3d;

}  // namespace quadrotor_mpc
