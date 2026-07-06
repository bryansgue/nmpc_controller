# Repository Scope

This package is now a nominal quadrotor NMPC controller.

## Active Code

- `ocp_generation/generate_nmpc_ocp.py`: the only OCP generator.
- `c_generated_code_nmpc/`: acados-generated NMPC solver code.
- `include/quadrotor_mpc/nmpc/` and `src/nmpc/`: C++ NMPC wrapper plus MiL/SiL executables.
- `include/quadrotor_mpc/common/`, `src/common/`: shared math and integration helpers.
- `include/quadrotor_mpc/trajectory/`, `src/trajectory/`: Lissajous flatness references and attitude reference generation.
- `include/quadrotor_mpc/mujoco/`, `src/mujoco/`: MuJoCo/ROS2 SiL bridge.

## Current NMPC Interface

The nominal NMPC tracks flatness references:

- desired position `p_ref`
- desired orientation `q_ref`
- desired velocity `v_ref`

The runtime parameter vector has 23 entries:

`[p_ref(3), q_ref(4), Q_pos(3), Q_att(3), R_u(4), v_ref(3), Q_vel(3)]`

The plant model is nominal:

- mass fixed in the generator
- no disturbance estimate
- no adaptive mass/time-constant parameters

## Build

Regenerate solver:

```bash
python3 ocp_generation/generate_nmpc_ocp.py
```

Build nominal executables:

```bash
cmake -S . -B build
cmake --build build --target nmpc_sim nmpc_sil -j2
```
