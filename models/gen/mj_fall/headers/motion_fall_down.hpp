#pragma once

#include "runtime.hpp"
#include "shared_state.hpp"

struct motion_fall_down_state {
    bool active = false;
    arm_solver_solver_state arm_solver;
    bool snapshot_taken = false;
    motion_spec::runtime::PIDControl ctrl_linvel_x{10.0, 0.0, 0.0};
    motion_spec::runtime::PIDControl ctrl_linvel_y{10.0, 0.0, 0.0};
    motion_spec::runtime::PIDControl ctrl_linvel_z{10.0, 0.0, 0.0};
    motion_spec::runtime::PIDControl ctrl_orientation_ang_x{10.0, 0.0, 6.0};
    motion_spec::runtime::PIDControl ctrl_orientation_ang_y{10.0, 0.0, 6.0};
    motion_spec::runtime::PIDControl ctrl_orientation_ang_z{10.0, 0.0, 6.0};
};

inline void reset_motion_fall_down(motion_fall_down_state &state) {
    state = motion_fall_down_state{};
}

inline void init_motion_fall_down(motion_fall_down_state &state, const robot_io &robot) {
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
        state.arm_solver.rnea = std::make_unique<KDL::ChainIdSolver_RNE>(*robot.arm_solver.chain, KDL::Vector(0.0, 0.0, -9.81));
        state.arm_solver.initialized = true;
    }
}

inline void update_motion_fall_down(
    motion_fall_down_state &state,
    shared_data &shared,
    const robot_io &robot) {
    init_motion_fall_down(state, robot);

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
            shared.pose_ee,
            motion_spec::runtime::find_segment_index(*robot.arm_solver.chain, "bracelet_link"));
    }

    {
        KDL::ChainFkSolverVel_recursive fk(*robot.arm_solver.chain);
        KDL::FrameVel tmp;
        fk.JntToCart(
            q_qd_arm_solver,
            tmp,
            motion_spec::runtime::find_segment_index(*robot.arm_solver.chain, "bracelet_link"));
        shared.twist_ee = tmp.deriv();
    }


    if (!state.snapshot_taken) {
        shared.pose_ee_start = shared.pose_ee;
        state.snapshot_taken = true;
    }
}

inline bool can_start_motion_fall_down(
    motion_fall_down_state &state,
    shared_data &shared) {
    return true;
}

inline void monitor_motion_fall_down(
    motion_fall_down_state &state,
    shared_data &shared) {
}

inline void control_motion_fall_down(
    motion_fall_down_state &state,
    shared_data &shared,
    const robot_io &robot) {
    // eval_fall_down_while_cstr_linvel_x
    shared.twist_ee_linear_x_err_fall_down = motion_spec::runtime::evaluate_equality_constraint(shared.vel_x_zero, shared.twist_ee.vel[0]);
    // eval_fall_down_while_cstr_linvel_y
    shared.twist_ee_linear_y_err_fall_down = motion_spec::runtime::evaluate_equality_constraint(shared.vel_y_zero, shared.twist_ee.vel[1]);
    // eval_fall_down_while_cstr_linvel_z
    shared.twist_ee_linear_z_err_fall_down = motion_spec::runtime::evaluate_equality_constraint(shared.vel_z_down, shared.twist_ee.vel[2]);
    // eval_pose_diff_ctrl_orientation
    shared.pose_diff_ctrl_orientation = KDL::diff(shared.pose_ee, shared.pose_ee_start);
    // ctrl_orientation_ang_z
    shared.eacc_ctrl_orientation_ang_z = state.ctrl_orientation_ang_z.control_with_vel(shared.pose_diff_ctrl_orientation.rot[2], shared.twist_ee.rot[2]);
    // ctrl_orientation_ang_y
    shared.eacc_ctrl_orientation_ang_y = state.ctrl_orientation_ang_y.control_with_vel(shared.pose_diff_ctrl_orientation.rot[1], shared.twist_ee.rot[1]);
    // ctrl_orientation_ang_x
    shared.eacc_ctrl_orientation_ang_x = state.ctrl_orientation_ang_x.control_with_vel(shared.pose_diff_ctrl_orientation.rot[0], shared.twist_ee.rot[0]);
    // ctrl_linvel_z
    shared.eacc_twist_ee_linear_z_fall_down = state.ctrl_linvel_z.control(shared.twist_ee_linear_z_err_fall_down);
    // ctrl_linvel_y
    shared.eacc_twist_ee_linear_y_fall_down = state.ctrl_linvel_y.control(shared.twist_ee_linear_y_err_fall_down);
    // ctrl_linvel_x
    shared.eacc_twist_ee_linear_x_fall_down = state.ctrl_linvel_x.control(shared.twist_ee_linear_x_err_fall_down);



    KDL::SetToZero(state.arm_solver.f_cstr);
    state.arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 0) = 1.0;
    state.arm_solver.e_acc(0) = shared.eacc_twist_ee_linear_x_fall_down;

    state.arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 1) = 1.0;
    state.arm_solver.e_acc(1) = shared.eacc_twist_ee_linear_y_fall_down;

    state.arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 2) = 1.0;
    state.arm_solver.e_acc(2) = shared.eacc_twist_ee_linear_z_fall_down;

    state.arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::X), 3) = 1.0;
    state.arm_solver.e_acc(3) = shared.eacc_ctrl_orientation_ang_x;

    state.arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Y), 4) = 1.0;
    state.arm_solver.e_acc(4) = shared.eacc_ctrl_orientation_ang_y;

    state.arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Z), 5) = 1.0;
    state.arm_solver.e_acc(5) = shared.eacc_ctrl_orientation_ang_z;
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

inline void apply_motion_fall_down(
    motion_fall_down_state &state,
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
