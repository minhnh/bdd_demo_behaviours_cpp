#pragma once

#include "runtime.hpp"
#include "shared_state.hpp"

struct motion_touch_down_state {
    arm_solver_solver_state arm_solver;
    bool snapshot_taken = false;
    motion_spec::runtime::PIDControl ctrl_move_down_z{1.0, 0.0, 0.1};
    motion_spec::runtime::PIDControl ctrl_keep_lin_x_zero{1.0, 0.0, 0.1};
    motion_spec::runtime::PIDControl ctrl_keep_lin_y_zero{1.0, 0.0, 0.1};
    motion_spec::runtime::PIDControl ctrl_keep_ang_x_zero{1.0, 0.0, 0.1};
    motion_spec::runtime::PIDControl ctrl_keep_ang_y_zero{1.0, 0.0, 0.1};
    motion_spec::runtime::PIDControl ctrl_keep_ang_z_zero{1.0, 0.0, 0.1};
    bool monitor_contact_table_previous = false;
};

inline void reset_motion_touch_down(motion_touch_down_state &state) {
    state = motion_touch_down_state{};
}

inline void init_motion_touch_down(motion_touch_down_state &state, const robot_io &robot) {
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

inline void update_motion_touch_down(
    motion_touch_down_state &state,
    shared_data &shared,
    const robot_io &robot) {
    init_motion_touch_down(state, robot);

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
        state.snapshot_taken = true;
    }
}

inline bool can_start_motion_touch_down(
    motion_touch_down_state &state,
    shared_data &shared) {
    return true;
}

inline void monitor_motion_touch_down(
    motion_touch_down_state &state,
    shared_data &shared) {
    // eval_touch_down_until_contact_table
    shared.twist_ee_base_linear_z_err = motion_spec::runtime::evaluate_equality_constraint(shared.twist_ee_base.vel[2], shared.zero_linvel);

    {
        const bool active = motion_spec::runtime::constraint_satisfied(shared.twist_ee_base_linear_z_err);
        if (motion_spec::runtime::rising_edge(state.monitor_contact_table_previous, active)) {
            motion_spec::runtime::warn_produce_event_not_implemented("table_contact");
        }
    }

}

inline void control_motion_touch_down(
    motion_touch_down_state &state,
    shared_data &shared,
    const robot_io &robot) {
    // eval_touch_down_while_move_down_z
    shared.twist_ee_base_linear_z_err_touch_down = motion_spec::runtime::evaluate_equality_constraint(shared.twist_ee_base.vel[2], shared.down_vel);
    // eval_touch_down_while_keep_lin_x_zero
    shared.twist_ee_base_linear_x_err = motion_spec::runtime::evaluate_equality_constraint(shared.twist_ee_base.vel[0], shared.zero_linvel);
    // eval_touch_down_while_keep_lin_y_zero
    shared.twist_ee_base_linear_y_err = motion_spec::runtime::evaluate_equality_constraint(shared.twist_ee_base.vel[1], shared.zero_linvel);
    // eval_touch_down_while_keep_ang_x_zero
    shared.twist_ee_base_angular_x_err = motion_spec::runtime::evaluate_equality_constraint(shared.twist_ee_base.rot[0], shared.zero_angvel);
    // eval_touch_down_while_keep_ang_y_zero
    shared.twist_ee_base_angular_y_err = motion_spec::runtime::evaluate_equality_constraint(shared.twist_ee_base.rot[1], shared.zero_angvel);
    // eval_touch_down_while_keep_ang_z_zero
    shared.twist_ee_base_angular_z_err = motion_spec::runtime::evaluate_equality_constraint(shared.twist_ee_base.rot[2], shared.zero_angvel);
    // ctrl_keep_ang_z_zero
    shared.eacc_twist_ee_base_angular_z = state.ctrl_keep_ang_z_zero.control(shared.twist_ee_base_angular_z_err);
    // ctrl_keep_ang_y_zero
    shared.eacc_twist_ee_base_angular_y = state.ctrl_keep_ang_y_zero.control(shared.twist_ee_base_angular_y_err);
    // ctrl_keep_ang_x_zero
    shared.eacc_twist_ee_base_angular_x = state.ctrl_keep_ang_x_zero.control(shared.twist_ee_base_angular_x_err);
    // ctrl_keep_lin_y_zero
    shared.eacc_twist_ee_base_linear_y = state.ctrl_keep_lin_y_zero.control(shared.twist_ee_base_linear_y_err);
    // ctrl_keep_lin_x_zero
    shared.eacc_twist_ee_base_linear_x = state.ctrl_keep_lin_x_zero.control(shared.twist_ee_base_linear_x_err);
    // ctrl_move_down_z
    shared.eacc_twist_ee_base_linear_z_touch_down = state.ctrl_move_down_z.control(shared.twist_ee_base_linear_z_err_touch_down);



    KDL::SetToZero(state.arm_solver.f_cstr);
    state.arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 0) = 1.0;
    state.arm_solver.e_acc(0) = shared.eacc_twist_ee_base_linear_z_touch_down;

    state.arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 1) = 1.0;
    state.arm_solver.e_acc(1) = shared.eacc_twist_ee_base_linear_x;

    state.arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 2) = 1.0;
    state.arm_solver.e_acc(2) = shared.eacc_twist_ee_base_linear_y;

    state.arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::X), 3) = 1.0;
    state.arm_solver.e_acc(3) = shared.eacc_twist_ee_base_angular_x;

    state.arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Y), 4) = 1.0;
    state.arm_solver.e_acc(4) = shared.eacc_twist_ee_base_angular_y;

    state.arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Z), 5) = 1.0;
    state.arm_solver.e_acc(5) = shared.eacc_twist_ee_base_angular_z;
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

inline void apply_motion_touch_down(
    motion_touch_down_state &state,
    shared_data &shared,
    const robot_io &robot) {
    for (int i = 0; i < state.arm_solver.num_joints; ++i) {
        robot.arm_solver.state->eff_cmd[i] = state.arm_solver.tau_ctrl(i);
    }
    robif2b_kinova_gen3_update(robot.arm_solver.robot);
}