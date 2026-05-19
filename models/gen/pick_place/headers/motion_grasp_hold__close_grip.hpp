#pragma once

#include "runtime.hpp"
#include "shared_state.hpp"

struct motion_grasp_hold__close_grip_state {
    bool active = false;
    int active_steps = 0;
    double trajectory_start_time = -1.0;
    arm_solver_solver_state arm_solver;
    bool snapshot_taken = false;
    motion_spec::runtime::PIDControl ctrl_cg_hold_x{100.0, 50.0, 20.0, 0.9};
    motion_spec::runtime::PIDControl ctrl_cg_hold_y{100.0, 50.0, 20.0, 0.9};
    motion_spec::runtime::PIDControl ctrl_cg_hold_z{100.0, 50.0, 20.0, 0.9};
    motion_spec::runtime::PIDControl ctrl_cg_hold_orientation_ang_x{120.0, 50.0, 80.0, 0.0};
    motion_spec::runtime::PIDControl ctrl_cg_hold_orientation_ang_y{120.0, 50.0, 80.0, 0.0};
    motion_spec::runtime::PIDControl ctrl_cg_hold_orientation_ang_z{120.0, 50.0, 80.0, 0.0};
    bool mon_at_cube_previous = false;
};

inline void reset_motion_grasp_hold__close_grip(motion_grasp_hold__close_grip_state &state) {
    state = motion_grasp_hold__close_grip_state{};
}

inline void init_motion_grasp_hold__close_grip(motion_grasp_hold__close_grip_state &state, const robot_io &robot) {
    if (!state.arm_solver.initialized) {
        state.arm_solver.num_constraints = 9;
        state.arm_solver.num_joints = robot.arm_solver.chain->getNrOfJoints();
        state.arm_solver.num_segments = robot.arm_solver.chain->getNrOfSegments();
        state.arm_solver.q = KDL::JntArray(state.arm_solver.num_joints);
        state.arm_solver.qd = KDL::JntArray(state.arm_solver.num_joints);
        state.arm_solver.qdd = KDL::JntArray(state.arm_solver.num_joints);
        state.arm_solver.tau_ff = KDL::JntArray(state.arm_solver.num_joints);
        state.arm_solver.tau_ctrl = KDL::JntArray(state.arm_solver.num_joints);
        state.arm_solver.f_ext = KDL::Wrenches(state.arm_solver.num_segments);
        state.arm_solver.f_cstr = KDL::Jacobian(state.arm_solver.num_constraints);
        state.arm_solver.e_acc = KDL::JntArray(state.arm_solver.num_constraints);
        state.arm_solver.root_acc.vel = KDL::Vector(0.0, 0.0, -9.81);
        state.arm_solver.achd_fext = std::make_unique<KDL::ChainHdSolver_Vereshchagin_Fext_FixedJoint>(*robot.arm_solver.chain, state.arm_solver.root_acc, state.arm_solver.num_constraints);
        state.arm_solver.achd_acc = std::make_unique<KDL::ChainHdSolver_Vereshchagin_Fixed_Joint>(*robot.arm_solver.chain, state.arm_solver.root_acc, state.arm_solver.num_constraints);
        state.arm_solver.rnea = std::make_unique<KDL::ChainIdSolver_RNE>(*robot.arm_solver.chain, KDL::Vector(-0.0, -0.0, 9.81));
        state.arm_solver.initialized = true;
    }
}

inline void update_motion_grasp_hold__close_grip(
    motion_grasp_hold__close_grip_state &state,
    shared_data &shared,
    const robot_io &robot) {
    init_motion_grasp_hold__close_grip(state, robot);

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
        KDL::Frame _body_frame_pose_cube_base;
        if (!mj_kdl::get_body_frame(
                robot.arm_solver.robot->model,
                robot.arm_solver.robot->data,
                "cube",
                &_body_frame_pose_cube_base)) {
            throw std::runtime_error("MuJoCo body not found for scene object pose output: cube");
        }
        KDL::Frame _base_world_frame_arm_solver;
        mj_kdl::get_body_frame(
                robot.arm_solver.robot->model,
                robot.arm_solver.robot->data,
                "base_link",
                &_base_world_frame_arm_solver);
        shared.pose_cube_base = _base_world_frame_arm_solver.Inverse() * _body_frame_pose_cube_base;
    }

    {
        KDL::ChainFkSolverPos_recursive fk(*robot.arm_solver.chain);
        fk.JntToCart(
            state.arm_solver.q,
            shared.pose_elbow_base,
            motion_spec::runtime::find_segment_index(*robot.arm_solver.chain, "half_arm_2_link"));
    }


    if (!state.snapshot_taken) {
        shared.hold_orientation_pose = shared.pose_ee_base;
        shared.hold_x = shared.pose_ee_base.p[0];
        shared.hold_y = shared.pose_ee_base.p[1];
        shared.hold_z = shared.pose_ee_base.p[2];
        shared.support_z = shared.pose_elbow_base.p[2];
        state.snapshot_taken = true;
    }
}

inline bool can_start_motion_grasp_hold__close_grip(
    motion_grasp_hold__close_grip_state &state,
    shared_data &shared) {
    return true;
}

inline void monitor_motion_grasp_hold__close_grip(
    motion_grasp_hold__close_grip_state &state,
    shared_data &shared) {
}

inline void control_motion_grasp_hold__close_grip(
    motion_grasp_hold__close_grip_state &state,
    shared_data &shared,
    const robot_io &robot) {
    if (state.trajectory_start_time < 0.0) {
        if (robot.arm_solver.robot != nullptr) {
        state.trajectory_start_time = robot.arm_solver.robot->data->time;
        }
    }

    // eval_grasp_hold_while_hold_x
    shared.pose_ee_base_distance_x_err_grasp_hold = motion_spec::runtime::evaluate_equality_constraint(shared.hold_x, shared.pose_ee_base.p[0]);
    // eval_grasp_hold_while_hold_y
    shared.pose_ee_base_distance_y_err_grasp_hold = motion_spec::runtime::evaluate_equality_constraint(shared.hold_y, shared.pose_ee_base.p[1]);
    // eval_grasp_hold_while_hold_z
    shared.pose_ee_base_distance_z_err_grasp_hold = motion_spec::runtime::evaluate_equality_constraint(shared.hold_z, shared.pose_ee_base.p[2]);
    // eval_pose_diff_ctrl_cg_hold_orientation
    shared.pose_diff_ctrl_cg_hold_orientation = KDL::diff(shared.pose_ee_base, shared.hold_orientation_pose);
    // ctrl_cg_hold_orientation_ang_z
    shared.eacc_ctrl_cg_hold_orientation_ang_z = state.ctrl_cg_hold_orientation_ang_z.control(shared.pose_diff_ctrl_cg_hold_orientation.rot[2]);
    // ctrl_cg_hold_orientation_ang_y
    shared.eacc_ctrl_cg_hold_orientation_ang_y = state.ctrl_cg_hold_orientation_ang_y.control(shared.pose_diff_ctrl_cg_hold_orientation.rot[1]);
    // ctrl_cg_hold_orientation_ang_x
    shared.eacc_ctrl_cg_hold_orientation_ang_x = state.ctrl_cg_hold_orientation_ang_x.control(shared.pose_diff_ctrl_cg_hold_orientation.rot[0]);
    // ctrl_cg_hold_z
    shared.eacc_pose_ee_base_distance_z_grasp_hold = state.ctrl_cg_hold_z.control(shared.pose_ee_base_distance_z_err_grasp_hold);
    // ctrl_cg_hold_y
    shared.eacc_pose_ee_base_distance_y_grasp_hold = state.ctrl_cg_hold_y.control(shared.pose_ee_base_distance_y_err_grasp_hold);
    // ctrl_cg_hold_x
    shared.eacc_pose_ee_base_distance_x_grasp_hold = state.ctrl_cg_hold_x.control(shared.pose_ee_base_distance_x_err_grasp_hold);


    {
        const bool active = motion_spec::runtime::constraint_satisfied(shared.pose_ee_base_distance_x_err);
        if (motion_spec::runtime::rising_edge(state.mon_at_cube_previous, active)) {
            motion_spec::runtime::warn_produce_event_not_implemented("at_grasp");
        }
    }


    KDL::SetToZero(state.arm_solver.f_cstr);
    state.arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 0) = 1.0;
    state.arm_solver.e_acc(0) = motion_spec::runtime::clamp_abs(shared.eacc_pose_ee_base_distance_x_grasp_hold, motion_spec::runtime::kBetaMaxLin);

    state.arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 1) = 1.0;
    state.arm_solver.e_acc(1) = motion_spec::runtime::clamp_abs(shared.eacc_pose_ee_base_distance_y_grasp_hold, motion_spec::runtime::kBetaMaxLin);

    state.arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 2) = 1.0;
    state.arm_solver.e_acc(2) = motion_spec::runtime::clamp_abs(shared.eacc_pose_ee_base_distance_z_grasp_hold, motion_spec::runtime::kBetaMaxLin);

    state.arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::X), 3) = 1.0;
    state.arm_solver.e_acc(3) = motion_spec::runtime::clamp_abs(shared.eacc_ctrl_cg_hold_orientation_ang_x, motion_spec::runtime::kBetaMaxRot);

    state.arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Y), 4) = 1.0;
    state.arm_solver.e_acc(4) = motion_spec::runtime::clamp_abs(shared.eacc_ctrl_cg_hold_orientation_ang_y, motion_spec::runtime::kBetaMaxRot);

    state.arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Z), 5) = 1.0;
    state.arm_solver.e_acc(5) = motion_spec::runtime::clamp_abs(shared.eacc_ctrl_cg_hold_orientation_ang_z, motion_spec::runtime::kBetaMaxRot);

    state.arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::X), 6) = 1.0;
    state.arm_solver.e_acc(6) = motion_spec::runtime::clamp_abs(shared.eacc_ctrl_og_hold_orientation_ang_x, motion_spec::runtime::kBetaMaxRot);

    state.arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Y), 7) = 1.0;
    state.arm_solver.e_acc(7) = motion_spec::runtime::clamp_abs(shared.eacc_ctrl_og_hold_orientation_ang_y, motion_spec::runtime::kBetaMaxRot);

    state.arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Z), 8) = 1.0;
    state.arm_solver.e_acc(8) = motion_spec::runtime::clamp_abs(shared.eacc_ctrl_og_hold_orientation_ang_z, motion_spec::runtime::kBetaMaxRot);
    KDL::SetToZero(state.arm_solver.tau_ff);
    for (int i = 0; i < state.arm_solver.num_segments; ++i) {
        KDL::SetToZero(state.arm_solver.f_ext[i]);
    }
    state.arm_solver.f_ext[motion_spec::runtime::find_segment_index(*robot.arm_solver.chain, "half_arm_2_link") - 1] += shared.wrench_force_ctrl_og_support_z;
    KDL::JntArray qdd_fext_arm_solver(state.arm_solver.num_joints);
    KDL::Jacobian alpha_zero_arm_solver(state.arm_solver.num_constraints);
    KDL::JntArray beta_zero_arm_solver(state.arm_solver.num_constraints);
    KDL::JntArray ff_zero_arm_solver(state.arm_solver.num_joints);
    KDL::SetToZero(alpha_zero_arm_solver);
    KDL::SetToZero(beta_zero_arm_solver);
    KDL::SetToZero(ff_zero_arm_solver);
    KDL::JntArray tau_cstr_fext_arm_solver(state.arm_solver.num_joints);
    state.arm_solver.achd_fext->CartToJnt(
        state.arm_solver.q,
        state.arm_solver.qd,
        qdd_fext_arm_solver,
        alpha_zero_arm_solver,
        beta_zero_arm_solver,
        state.arm_solver.f_ext,
        ff_zero_arm_solver,
        tau_cstr_fext_arm_solver);
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
    KDL::JntArray tau_fext_arm_solver(state.arm_solver.num_joints);
    state.arm_solver.rnea->CartToJnt(
        state.arm_solver.q,
        state.arm_solver.qd,
        qdd_fext_arm_solver,
        f_ext_zero_arm_solver,
        tau_fext_arm_solver);
    KDL::Add(state.arm_solver.tau_ctrl, tau_fext_arm_solver, state.arm_solver.tau_ctrl);

}

inline void apply_motion_grasp_hold__close_grip(
    motion_grasp_hold__close_grip_state &state,
    shared_data &shared,
    const robot_io &robot) {
    for (int i = 0; i < state.arm_solver.num_joints; ++i) {
        robot.arm_solver.robot->jnt_trq_cmd[i] = motion_spec::runtime::clamp_abs(state.arm_solver.tau_ctrl(i), motion_spec::runtime::kTauMax);
    }
    mj_kdl::update(robot.arm_solver.robot);
    if (!mj_kdl::step(robot.arm_solver.robot)) {
        std::exit(0);
    }
    {
        const int actuator_id = mj_name2id(robot.arm_solver.robot->model, mjOBJ_ACTUATOR, "g_fingers_actuator");
        if (actuator_id >= 0) {
            robot.arm_solver.robot->data->ctrl[actuator_id] = 255.0;
        }
    }

}
