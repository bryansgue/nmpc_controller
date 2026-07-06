#!/usr/bin/env python3
import argparse
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def quat_angle_error(q, qr):
    dots = np.abs(np.sum(q * qr, axis=1))
    dots = np.clip(dots, -1.0, 1.0)
    return 2.0 * np.arccos(dots)


def quat_to_yaw(q):
    qw, qx, qy, qz = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    return np.unwrap(np.arctan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    ))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "csv",
        nargs="?",
        default="/home/bryansgue/robotics/controller_ws/nmpc_sil_results.csv",
    )
    parser.add_argument("--out", default="nmpc_sil_diagnostics.png")
    args = parser.parse_args()

    data = np.genfromtxt(args.csv, delimiter=",", names=True)
    if data.ndim == 0:
        raise RuntimeError(f"CSV has no samples: {args.csv}")

    t = data["t"]
    p = np.column_stack([data["px"], data["py"], data["pz"]])
    v = np.column_stack([data["vx"], data["vy"], data["vz"]])
    q = np.column_stack([data["qw"], data["qx"], data["qy"], data["qz"]])
    w = np.column_stack([data["wx"], data["wy"], data["wz"]])
    u = np.column_stack([data["T"], data["wx_cmd"], data["wy_cmd"], data["wz_cmd"]])
    pr = np.column_stack([data["px_ref"], data["py_ref"], data["pz_ref"]])
    qr = np.column_stack([data["qw_ref"], data["qx_ref"], data["qy_ref"], data["qz_ref"]])
    status = data["status"].astype(int)

    e_pos = p - pr
    e_norm = np.linalg.norm(e_pos, axis=1)
    att_err_deg = np.rad2deg(quat_angle_error(q, qr))
    yaw = quat_to_yaw(q)
    yaw_ref = quat_to_yaw(qr)
    yaw_err = np.rad2deg(np.unwrap(yaw - yaw_ref))

    fig, ax = plt.subplots(4, 2, figsize=(15, 12), sharex=True)
    fig.suptitle(f"NMPC SiL diagnostics: {Path(args.csv).name}")

    ax[0, 0].plot(t, e_pos[:, 0], label="ex")
    ax[0, 0].plot(t, e_pos[:, 1], label="ey")
    ax[0, 0].plot(t, e_pos[:, 2], label="ez")
    ax[0, 0].plot(t, e_norm, "k", label="|e|", linewidth=1.6)
    ax[0, 0].set_ylabel("pos error [m]")
    ax[0, 0].legend(loc="upper right")
    ax[0, 0].grid(True)

    ax[0, 1].plot(t, att_err_deg, label="quat angle")
    ax[0, 1].plot(t, yaw_err, label="yaw err")
    ax[0, 1].set_ylabel("att error [deg]")
    ax[0, 1].legend(loc="upper right")
    ax[0, 1].grid(True)

    labels = ["x", "y", "z"]
    for i, lab in enumerate(labels):
        ax[1, 0].plot(t, p[:, i], label=f"p{lab}")
        ax[1, 0].plot(t, pr[:, i], "--", label=f"p{lab}_ref")
    ax[1, 0].set_ylabel("position [m]")
    ax[1, 0].legend(ncol=3, fontsize=8)
    ax[1, 0].grid(True)

    for i, lab in enumerate(labels):
        ax[1, 1].plot(t, v[:, i], label=f"v{lab}")
    ax[1, 1].set_ylabel("velocity [m/s]")
    ax[1, 1].legend(loc="upper right")
    ax[1, 1].grid(True)

    ax[2, 0].plot(t, u[:, 0], label="T")
    ax[2, 0].axhline(1.05 * 9.81, color="k", linestyle="--", linewidth=1, label="hover")
    ax[2, 0].axhline(49.05, color="r", linestyle=":", linewidth=1, label="T max")
    ax[2, 0].set_ylabel("thrust [N]")
    ax[2, 0].legend(loc="upper right")
    ax[2, 0].grid(True)

    ax[2, 1].plot(t, u[:, 1], label="wx_cmd")
    ax[2, 1].plot(t, u[:, 2], label="wy_cmd")
    ax[2, 1].plot(t, u[:, 3], label="wz_cmd")
    ax[2, 1].axhline(20.0, color="r", linestyle=":", linewidth=1)
    ax[2, 1].axhline(-20.0, color="r", linestyle=":", linewidth=1)
    ax[2, 1].set_ylabel("rate cmd [rad/s]")
    ax[2, 1].legend(loc="upper right")
    ax[2, 1].grid(True)

    ax[3, 0].plot(t, w[:, 0], label="wx")
    ax[3, 0].plot(t, w[:, 1], label="wy")
    ax[3, 0].plot(t, w[:, 2], label="wz")
    ax[3, 0].set_ylabel("body rate [rad/s]")
    ax[3, 0].set_xlabel("time [s]")
    ax[3, 0].legend(loc="upper right")
    ax[3, 0].grid(True)

    ax[3, 1].plot(t, data["solve_ms"], label="solve")
    ax[3, 1].plot(t, data["loop_ms"], label="loop")
    bad = status != 0
    if np.any(bad):
        ax[3, 1].scatter(t[bad], data["solve_ms"][bad], color="r", s=12, label="status != 0")
    ax[3, 1].set_ylabel("time [ms]")
    ax[3, 1].set_xlabel("time [s]")
    ax[3, 1].legend(loc="upper right")
    ax[3, 1].grid(True)

    fig.tight_layout()
    fig.savefig(args.out, dpi=160)

    print(f"samples: {len(t)}")
    print(f"t: {t[0]:.2f} -> {t[-1]:.2f} s")
    print(f"status counts: {dict(zip(*np.unique(status, return_counts=True)))}")
    print(f"pos error mean/max: {np.mean(e_norm):.3f} / {np.max(e_norm):.3f} m")
    print(f"att error mean/max: {np.mean(att_err_deg):.2f} / {np.max(att_err_deg):.2f} deg")
    print(f"thrust min/mean/max: {np.min(u[:,0]):.3f} / {np.mean(u[:,0]):.3f} / {np.max(u[:,0]):.3f} N")
    print(f"T low count (<0.05 N): {int(np.sum(u[:,0] < 0.05))}")
    print(f"rate max: {np.max(np.abs(u[:,1:])):.3f} rad/s")
    print(f"plot: {args.out}")


if __name__ == "__main__":
    main()
