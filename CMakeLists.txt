cmake_minimum_required(VERSION 3.14)
project(quadrotor_nmpc LANGUAGES CXX C)

# Default to Release (-O3) — solver needs it to meet 10ms budget at 100 Hz
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ── Dependencies ─────────────────────────────────────────────────────────────
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(nlohmann_json 3.2 REQUIRED)

# acados
if(DEFINED ENV{ACADOS_SOURCE_DIR})
    set(ACADOS_DIR $ENV{ACADOS_SOURCE_DIR})
else()
    set(ACADOS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../acados")
endif()
message(STATUS "ACADOS_DIR = ${ACADOS_DIR}")

set(ACADOS_INCLUDE_DIR ${ACADOS_DIR}/include)
set(ACADOS_LIB_DIR     ${ACADOS_DIR}/lib)

# ROS2 (optional — only needed for SiL targets)
find_package(ament_cmake QUIET)
find_package(rclcpp QUIET)
find_package(nav_msgs QUIET)
find_package(sensor_msgs QUIET)
find_package(mujoco_ros_utils QUIET)
find_package(std_msgs QUIET)
find_package(std_srvs QUIET)
find_package(geometry_msgs QUIET)
find_package(quadrotor_msgs QUIET)
find_package(mav_manager_srv QUIET)

if(rclcpp_FOUND AND quadrotor_msgs_FOUND)
    set(HAS_ROS2 TRUE)
    message(STATUS "ROS2 found — SiL targets will be built")
else()
    set(HAS_ROS2 FALSE)
    message(STATUS "ROS2 NOT found — only MiL targets will be built")
endif()

# ── Common library (quaternion, RK4, params) ─────────────────────────────────
add_library(quadrotor_common STATIC
    src/common/quaternion_algebra.cpp
    src/common/integrators.cpp
)
target_include_directories(quadrotor_common PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${ACADOS_INCLUDE_DIR}
    ${ACADOS_INCLUDE_DIR}/acados
    ${ACADOS_INCLUDE_DIR}/blasfeo/include
    ${ACADOS_INCLUDE_DIR}/hpipm/include
)
target_link_libraries(quadrotor_common PUBLIC Eigen3::Eigen)

# ── Trajectory library (Lissajous, arc-length, attitude ref) ─────────────────
add_library(quadrotor_trajectory STATIC
    src/trajectory/lissajous.cpp
    src/trajectory/arc_length.cpp
    src/trajectory/attitude_reference.cpp
)
target_link_libraries(quadrotor_trajectory PUBLIC quadrotor_common)

# ── MuJoCo ROS2 interface (shared by all SiL controllers) ───────────────────
if(HAS_ROS2)
    add_library(mujoco_interface STATIC
        src/mujoco/mujoco_interface.cpp
        src/mujoco/sil_protocol.cpp
    )
    target_link_libraries(mujoco_interface PUBLIC
        quadrotor_common
    )
    ament_target_dependencies(mujoco_interface PUBLIC
        rclcpp nav_msgs sensor_msgs mujoco_ros_utils std_msgs std_srvs geometry_msgs quadrotor_msgs
    )
endif()

# ── NMPC solver (generated C code — added after running ocp_generation/) ─────
set(NMPC_CODEGEN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/c_generated_code_nmpc)
if(EXISTS ${NMPC_CODEGEN_DIR}/acados_solver_quadrotor_nmpc.c)
    message(STATUS "NMPC generated code found at ${NMPC_CODEGEN_DIR}")

    file(GLOB NMPC_MODEL_SRC   ${NMPC_CODEGEN_DIR}/quadrotor_nmpc_model/*.c)
    file(GLOB NMPC_COST_SRC    ${NMPC_CODEGEN_DIR}/quadrotor_nmpc_cost/*.c)
    file(GLOB NMPC_CONSTR_SRC  ${NMPC_CODEGEN_DIR}/quadrotor_nmpc_constraints/*.c)

    add_library(nmpc_solver STATIC
        ${NMPC_CODEGEN_DIR}/acados_solver_quadrotor_nmpc.c
        ${NMPC_MODEL_SRC}
        ${NMPC_COST_SRC}
        ${NMPC_CONSTR_SRC}
    )
    target_include_directories(nmpc_solver PUBLIC
        ${NMPC_CODEGEN_DIR}
        ${ACADOS_INCLUDE_DIR}
        ${ACADOS_INCLUDE_DIR}/acados
        ${ACADOS_INCLUDE_DIR}/blasfeo/include
        ${ACADOS_INCLUDE_DIR}/hpipm/include
    )
    target_link_libraries(nmpc_solver PUBLIC
        ${ACADOS_LIB_DIR}/libacados.so
        ${ACADOS_LIB_DIR}/libblasfeo.so
        ${ACADOS_LIB_DIR}/libhpipm.so
        m
    )

    # ── NMPC MiL simulation (standalone, no ROS2) ────────────────────
    add_executable(nmpc_sim
        src/nmpc/nmpc_sim.cpp
        src/nmpc/nmpc_controller.cpp
    )
    target_link_libraries(nmpc_sim PRIVATE
        quadrotor_common
        quadrotor_trajectory
        nmpc_solver
    )

    # ── NMPC SiL simulation (MuJoCo + ROS2) ─────────────────────────
    if(HAS_ROS2)
        add_executable(nmpc_sil
            src/nmpc/nmpc_sil.cpp
            src/nmpc/nmpc_controller.cpp
        )
        target_link_libraries(nmpc_sil
            quadrotor_common
            quadrotor_trajectory
            nmpc_solver
            mujoco_interface
        )
        ament_target_dependencies(nmpc_sil
            rclcpp nav_msgs std_msgs std_srvs geometry_msgs quadrotor_msgs
        )
    endif()

else()
    message(STATUS "NMPC generated code NOT found — run ocp_generation/generate_nmpc_ocp.py first")
endif()

# ── Install targets (for colcon) ─────────────────────────────────────────────
if(ament_cmake_FOUND)
    set(_install_targets "")
    foreach(_t nmpc_sil nmpc_sim)
        if(TARGET ${_t})
            list(APPEND _install_targets ${_t})
        endif()
    endforeach()
    if(_install_targets)
        install(TARGETS ${_install_targets}
            DESTINATION lib/${PROJECT_NAME}
        )
    endif()

    # Config YAML (para --params-file)
    install(DIRECTORY config/
        DESTINATION share/${PROJECT_NAME}/config
    )

    ament_package()
endif()
