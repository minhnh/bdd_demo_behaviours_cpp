#pragma once

#include "runtime.hpp"
#include "shared_state.hpp"

struct motion_idle_state {
    arm_solver_solver_state arm_solver;
    bool snapshot_taken = false;
    motion_spec::runtime::PIDControl ctrl_hold_pose_lin_x{4.0, 0.0, 0.4};
    motion_spec::runtime::PIDControl ctrl_hold_pose_lin_y{4.0, 0.0, 0.4};
    motion_spec::runtime::PIDControl ctrl_hold_pose_lin_z{4.0, 0.0, 0.4};
    motion_spec::runtime::PIDControl ctrl_hold_pose_ang_x{4.0, 0.0, 0.4};
    motion_spec::runtime::PIDControl ctrl_hold_pose_ang_y{4.0, 0.0, 0.4};
    motion_spec::runtime::PIDControl ctrl_hold_pose_ang_z{4.0, 0.0, 0.4};
};

inline void reset_motion_idle(motion_idle_state &state) {
    state = motion_idle_state{};
}

inline void init_motion_idle(motion_idle_state &state, const robot_io &robot) {
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
        state.arm_solver.achd_acc = std::make_unique<KDL::ChainHdSolver_Vereshchagin>(*robot.arm_solver.chain, state.arm_solver.root_acc, state.arm_solver.num_constraints);
        state.arm_solver.initialized = true;
    }
}

inline void update_motion_idle(
    motion_idle_state &state,
    shared_data &shared,
    const robot_io &robot) {
    init_motion_idle(state, robot);

    for (int i = 0; i < state.arm_solver.num_joints; ++i) {
        state.arm_solver.q(i) = robot.arm_solver.state->pos_msr[i];
        state.arm_solver.qd(i) = robot.arm_solver.state->vel_msr[i];
    }
    KDL::JntArrayVel q_qd_arm_solver(state.arm_solver.q, state.arm_solver.qd);
    {
        KDL::ChainFkSolverVel_recursive fk(*robot.arm_solver.chain);
        KDL::FrameVel tmp;
        fk.JntToCart(
            q_qd_arm_solver,
            tmp,
            motion_spec::runtime::find_segment_index(*robot.arm_solver.chain, "link_ee"));
        shared.twist_ee_base = tmp.deriv();
    }

    {
        KDL::ChainFkSolverPos_recursive fk(*robot.arm_solver.chain);
        fk.JntToCart(
            state.arm_solver.q,
            shared.pose_ee_base,
            motion_spec::runtime::find_segment_index(*robot.arm_solver.chain, "frame_ee"));
    }


    if (!state.snapshot_taken) {
        shared.pose_ee_start = shared.pose_ee_base;
        state.snapshot_taken = true;
    }
}

inline bool can_start_motion_idle(
    motion_idle_state &state,
    shared_data &shared) {
    return true;
}

inline void monitor_motion_idle(
    motion_idle_state &state,
    shared_data &shared) {
}

inline void control_motion_idle(
    motion_idle_state &state,
    shared_data &shared,
    const robot_io &robot) {
    // eval_pose_diff_ctrl_hold_pose
    shared.pose_diff_ctrl_hold_pose = KDL::diff(shared.pose_ee_start, shared.pose_ee_base);
    // ctrl_hold_pose_ang_z
    shared.eacc_ctrl_hold_pose_ang_z = state.ctrl_hold_pose_ang_z.control(shared.pose_diff_ctrl_hold_pose.rot[2]);
    // ctrl_hold_pose_ang_y
    shared.eacc_ctrl_hold_pose_ang_y = state.ctrl_hold_pose_ang_y.control(shared.pose_diff_ctrl_hold_pose.rot[1]);
    // ctrl_hold_pose_ang_x
    shared.eacc_ctrl_hold_pose_ang_x = state.ctrl_hold_pose_ang_x.control(shared.pose_diff_ctrl_hold_pose.rot[0]);
    // ctrl_hold_pose_lin_z
    shared.eacc_ctrl_hold_pose_lin_z = state.ctrl_hold_pose_lin_z.control(shared.pose_diff_ctrl_hold_pose.vel[2]);
    // ctrl_hold_pose_lin_y
    shared.eacc_ctrl_hold_pose_lin_y = state.ctrl_hold_pose_lin_y.control(shared.pose_diff_ctrl_hold_pose.vel[1]);
    // ctrl_hold_pose_lin_x
    shared.eacc_ctrl_hold_pose_lin_x = state.ctrl_hold_pose_lin_x.control(shared.pose_diff_ctrl_hold_pose.vel[0]);



    KDL::SetToZero(state.arm_solver.f_cstr);
    state.arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 0) = 1.0;
    state.arm_solver.e_acc(0) = shared.eacc_ctrl_hold_pose_lin_x;
    state.arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 1) = 1.0;
    state.arm_solver.e_acc(1) = shared.eacc_ctrl_hold_pose_lin_y;
    state.arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 2) = 1.0;
    state.arm_solver.e_acc(2) = shared.eacc_ctrl_hold_pose_lin_z;
    state.arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::X), 3) = 1.0;
    state.arm_solver.e_acc(3) = shared.eacc_ctrl_hold_pose_ang_x;
    state.arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Y), 4) = 1.0;
    state.arm_solver.e_acc(4) = shared.eacc_ctrl_hold_pose_ang_y;
    state.arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Z), 5) = 1.0;
    state.arm_solver.e_acc(5) = shared.eacc_ctrl_hold_pose_ang_z;
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
    state.arm_solver.tau_ctrl = tau_ctrl_acc_arm_solver;

}

inline void apply_motion_idle(
    motion_idle_state &state,
    shared_data &shared,
    const robot_io &robot) {
    for (int i = 0; i < state.arm_solver.num_joints; ++i) {
        robot.arm_solver.state->eff_cmd[i] = state.arm_solver.tau_ctrl(i);
    }
    robif2b_kinova_gen3_update(robot.arm_solver.robot);
}