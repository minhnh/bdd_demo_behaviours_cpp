#pragma once

#include "runtime.hpp"
#include "shared_state.hpp"

struct motion_home_state {
    bool active = false;
    arm_solver_solver_state arm_solver;
    bool snapshot_taken = false;
    motion_spec::runtime::PIDControl ctrl_hold_position_lin_x{100.0, 10.0, 20.0, 0.9};
    motion_spec::runtime::PIDControl ctrl_hold_position_lin_y{100.0, 10.0, 20.0, 0.9};
    motion_spec::runtime::PIDControl ctrl_hold_position_lin_z{100.0, 10.0, 20.0, 0.9};
    motion_spec::runtime::PIDControl ctrl_hold_orientation_ang_x{80.0, 5.0, 40.0, 0.9};
    motion_spec::runtime::PIDControl ctrl_hold_orientation_ang_y{80.0, 5.0, 40.0, 0.9};
    motion_spec::runtime::PIDControl ctrl_hold_orientation_ang_z{80.0, 5.0, 40.0, 0.9};
    bool mon_home_settled_previous = false;
};

inline void reset_motion_home(motion_home_state &state) {
    state = motion_home_state{};
}

inline void init_motion_home(motion_home_state &state, const robot_io &robot) {
    if (!state.arm_solver.initialized) {
        state.arm_solver.num_constraints = 6;
        state.arm_solver.num_joints = robot.arm_solver.chain->getNrOfJoints();
        state.arm_solver.num_segments = robot.arm_solver.chain->getNrOfSegments();
        state.arm_solver.q = KDL::JntArray(state.arm_solver.num_joints);
        state.arm_solver.qd = KDL::JntArray(state.arm_solver.num_joints);
        state.arm_solver.qdd = KDL::JntArray(state.arm_solver.num_joints);
        state.arm_solver.tau_ff = KDL::JntArray(state.arm_solver.num_joints);
        state.arm_solver.tau_ctrl = KDL::JntArray(state.arm_solver.num_joints);
        state.arm_solver.f_cstr = KDL::Jacobian(state.arm_solver.num_constraints);
        state.arm_solver.e_acc = KDL::JntArray(state.arm_solver.num_constraints);
        state.arm_solver.root_acc.vel = KDL::Vector(-0.0, -0.0, 9.81);
        state.arm_solver.achd_acc = std::make_unique<KDL::ChainHdSolver_Vereshchagin_Fixed_Joint>(*robot.arm_solver.chain, state.arm_solver.root_acc, state.arm_solver.num_constraints);
        state.arm_solver.rnea = std::make_unique<KDL::ChainIdSolver_RNE>(*robot.arm_solver.chain, KDL::Vector());
        state.arm_solver.initialized = true;
    }
}

inline void update_motion_home(
    motion_home_state &state,
    shared_data &shared,
    const robot_io &robot) {
    init_motion_home(state, robot);

    mj_kdl::update(robot.arm_solver.robot);
    for (int i = 0; i < state.arm_solver.num_joints; ++i) {
        state.arm_solver.q(i) = robot.arm_solver.robot->jnt_pos_msr[i];
        state.arm_solver.qd(i) = robot.arm_solver.robot->jnt_vel_msr[i];
    }
    KDL::JntArrayVel q_qd_arm_solver(state.arm_solver.q, state.arm_solver.qd);
    {
        KDL::ChainFkSolverPos_recursive fk(*robot.arm_solver.chain);
        fk.JntToCart(
            state.arm_solver.q,
            shared.pose_ee_base,
            motion_spec::runtime::find_segment_index(*robot.arm_solver.chain, "g_pinch"));
    }

    {
        KDL::ChainFkSolverVel_recursive fk(*robot.arm_solver.chain);
        KDL::FrameVel tmp;
        fk.JntToCart(
            q_qd_arm_solver,
            tmp,
            motion_spec::runtime::find_segment_index(*robot.arm_solver.chain, "g_pinch"));
        shared.twist_ee_base = tmp.deriv();
    }

    {
        if (!mj_kdl::get_body_frame(
                robot.arm_solver.robot->model,
                robot.arm_solver.robot->data,
                "cube",
                &shared.pose_cube_base)) {
            throw std::runtime_error("MuJoCo body not found for scene object pose output: cube");
        }
    }

    {
        KDL::ChainFkSolverPos_recursive fk(*robot.arm_solver.chain);
        fk.JntToCart(
            state.arm_solver.q,
            shared.pose_elbow_base,
            motion_spec::runtime::find_segment_index(*robot.arm_solver.chain, "half_arm_2_link"));
    }


    if (!state.snapshot_taken) {
        shared.home_pose = shared.pose_ee_base;
        state.snapshot_taken = true;
    }
    shared.pose_ee_wrt_cube = shared.pose_cube_base.Inverse() * shared.pose_ee_base;

}

inline bool can_start_motion_home(
    motion_home_state &state,
    shared_data &shared) {
    return true;
}

inline void monitor_motion_home(
    motion_home_state &state,
    shared_data &shared) {
    // eval_home_until_lin_z_settled
    shared.twist_ee_base_linear_z_err = motion_spec::runtime::evaluate_equality_constraint(shared.zero_linvel, shared.twist_ee_base.vel[2]);

    {
        const bool active = motion_spec::runtime::constraint_satisfied(shared.twist_ee_base_linear_z_err);
        if (motion_spec::runtime::rising_edge(state.mon_home_settled_previous, active)) {
            motion_spec::runtime::warn_produce_event_not_implemented("home_settled");
        }
    }

}

inline void control_motion_home(
    motion_home_state &state,
    shared_data &shared,
    const robot_io &robot) {
    // eval_pose_diff_ctrl_hold_position
    shared.pose_diff_ctrl_hold_position = KDL::diff(shared.pose_ee_base, shared.home_pose);
    // eval_pose_diff_ctrl_hold_orientation
    shared.pose_diff_ctrl_hold_orientation = KDL::diff(shared.pose_ee_base, shared.home_pose);
    // ctrl_hold_orientation_ang_z
    shared.eacc_ctrl_hold_orientation_ang_z = state.ctrl_hold_orientation_ang_z.control(shared.pose_diff_ctrl_hold_orientation.rot[2]);
    // ctrl_hold_orientation_ang_y
    shared.eacc_ctrl_hold_orientation_ang_y = state.ctrl_hold_orientation_ang_y.control(shared.pose_diff_ctrl_hold_orientation.rot[1]);
    // ctrl_hold_orientation_ang_x
    shared.eacc_ctrl_hold_orientation_ang_x = state.ctrl_hold_orientation_ang_x.control(shared.pose_diff_ctrl_hold_orientation.rot[0]);
    // ctrl_hold_position_lin_z
    shared.eacc_ctrl_hold_position_lin_z = state.ctrl_hold_position_lin_z.control(shared.pose_diff_ctrl_hold_position.vel[2]);
    // ctrl_hold_position_lin_y
    shared.eacc_ctrl_hold_position_lin_y = state.ctrl_hold_position_lin_y.control(shared.pose_diff_ctrl_hold_position.vel[1]);
    // ctrl_hold_position_lin_x
    shared.eacc_ctrl_hold_position_lin_x = state.ctrl_hold_position_lin_x.control(shared.pose_diff_ctrl_hold_position.vel[0]);



    KDL::SetToZero(state.arm_solver.f_cstr);
    state.arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 0) = 1.0;
    state.arm_solver.e_acc(0) = shared.eacc_ctrl_hold_position_lin_x;

    state.arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 1) = 1.0;
    state.arm_solver.e_acc(1) = shared.eacc_ctrl_hold_position_lin_y;

    state.arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 2) = 1.0;
    state.arm_solver.e_acc(2) = shared.eacc_ctrl_hold_position_lin_z;

    state.arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::X), 3) = 1.0;
    state.arm_solver.e_acc(3) = shared.eacc_ctrl_hold_orientation_ang_x;

    state.arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Y), 4) = 1.0;
    state.arm_solver.e_acc(4) = shared.eacc_ctrl_hold_orientation_ang_y;

    state.arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Z), 5) = 1.0;
    state.arm_solver.e_acc(5) = shared.eacc_ctrl_hold_orientation_ang_z;
    KDL::SetToZero(state.arm_solver.tau_ff);
    KDL::Wrenches f_ext_zero_arm_solver(state.arm_solver.num_segments);
    KDL::JntArray tau_ctrl_acc_arm_solver(state.arm_solver.num_joints);
    state.arm_solver.achd_acc->CartToJnt(
        state.arm_solver.q,
        state.arm_solver.qd,
        state.arm_solver.qdd,
        state.arm_solver.f_cstr,
        state.arm_solver.e_acc,
        f_ext_zero_arm_solver,
        state.arm_solver.tau_ff,
        tau_ctrl_acc_arm_solver);
    state.arm_solver.rnea->CartToJnt(
        state.arm_solver.q,
        state.arm_solver.qd,
        state.arm_solver.qdd,
        f_ext_zero_arm_solver,
        state.arm_solver.tau_ctrl);

}

inline void apply_motion_home(
    motion_home_state &state,
    shared_data &shared,
    const robot_io &robot) {
    for (int i = 0; i < state.arm_solver.num_joints; ++i) {
        robot.arm_solver.robot->jnt_trq_cmd[i] = state.arm_solver.tau_ctrl(i);
    }
    mj_kdl::update(robot.arm_solver.robot);
    if (!mj_kdl::step(robot.arm_solver.robot)) {
        std::exit(0);
    }

}
