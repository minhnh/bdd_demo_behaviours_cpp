#pragma once

#include "runtime.hpp"
#include "shared_state.hpp"

struct motion_pick_above_state {
    bool active = false;
    int active_steps = 0;
    double trajectory_start_time = -1.0;
    arm_solver_solver_state arm_solver;
    bool snapshot_taken = false;
    motion_spec::runtime::PIDControl ctrl_pa_follow_pos_lin_x{200.0, 100.0, 40.0, 0.0};
    motion_spec::runtime::PIDControl ctrl_pa_follow_pos_lin_y{200.0, 100.0, 40.0, 0.0};
    motion_spec::runtime::PIDControl ctrl_pa_follow_pos_lin_z{200.0, 100.0, 40.0, 0.0};
    motion_spec::runtime::PIDControl ctrl_pa_follow_ori_ang_x{120.0, 50.0, 80.0, 0.0};
    motion_spec::runtime::PIDControl ctrl_pa_follow_ori_ang_y{120.0, 50.0, 80.0, 0.0};
    motion_spec::runtime::PIDControl ctrl_pa_follow_ori_ang_z{120.0, 50.0, 80.0, 0.0};
    bool mon_pick_above_settled_previous = false;
};

inline void reset_motion_pick_above(motion_pick_above_state &state) {
    state = motion_pick_above_state{};
}

inline void init_motion_pick_above(motion_pick_above_state &state, const robot_io &robot) {
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
        state.arm_solver.root_acc.vel = KDL::Vector(0.0, 0.0, -9.81);
        state.arm_solver.achd_acc = std::make_unique<KDL::ChainHdSolver_Vereshchagin_Fixed_Joint>(*robot.arm_solver.chain, state.arm_solver.root_acc, state.arm_solver.num_constraints);
        state.arm_solver.rnea = std::make_unique<KDL::ChainIdSolver_RNE>(*robot.arm_solver.chain, KDL::Vector(-0.0, -0.0, 9.81));
        state.arm_solver.initialized = true;
    }
}

inline void update_motion_pick_above(
    motion_pick_above_state &state,
    shared_data &shared,
    const robot_io &robot) {
    init_motion_pick_above(state, robot);

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
        shared.start_cube_x = shared.pose_cube_base.p[0];
        shared.start_cube_y = shared.pose_cube_base.p[1];
        shared.start_pose = shared.pose_ee_base;
        state.snapshot_taken = true;
    }
}

inline bool can_start_motion_pick_above(
    motion_pick_above_state &state,
    shared_data &shared) {
    return true;
}

inline void monitor_motion_pick_above(
    motion_pick_above_state &state,
    shared_data &shared) {
    // eval_pick_above_until_at_pick_above_settled
    shared.pose_ee_base_pose_err = motion_spec::runtime::evaluate_equality_constraint(shared.goal_pose, shared.pose_ee_base);

    {
        const bool active = motion_spec::runtime::constraint_satisfied(shared.pose_ee_base_pose_err);
        if (motion_spec::runtime::rising_edge(state.mon_pick_above_settled_previous, active)) {
            motion_spec::runtime::warn_produce_event_not_implemented("pick_above_settled");
        }
    }

}

inline void control_motion_pick_above(
    motion_pick_above_state &state,
    shared_data &shared,
    const robot_io &robot) {
    if (state.trajectory_start_time < 0.0) {
        if (robot.arm_solver.robot != nullptr) {
        state.trajectory_start_time = robot.arm_solver.robot->data->time;
        }
    }
    shared.alpha = motion_spec::runtime::clamp01((robot.arm_solver.robot->data->time - state.trajectory_start_time) / motion_spec::runtime::kDefaultTrajectoryDuration);

    // lerp_traj
    {
        const KDL::Frame _lerp_goal_traj = KDL::Frame(KDL::Rotation::RPY(shared.grasp_roll, shared.grasp_pitch, shared.grasp_yaw), KDL::Vector(shared.start_cube_x, shared.start_cube_y, shared.above_z));
        shared.goal_pose = _lerp_goal_traj;
        shared.traj = KDL::addDelta(shared.start_pose, KDL::diff(shared.start_pose, _lerp_goal_traj), motion_spec::runtime::smoothstep(shared.alpha));
    }
    // eval_pose_diff_ctrl_pa_follow_pos
    shared.pose_diff_ctrl_pa_follow_pos = KDL::diff(shared.pose_ee_base, shared.traj);
    // eval_pose_diff_ctrl_pa_follow_ori
    shared.pose_diff_ctrl_pa_follow_ori = KDL::diff(shared.pose_ee_base, shared.traj);
    // ctrl_pa_follow_ori_ang_z
    shared.eacc_ctrl_pa_follow_ori_ang_z = state.ctrl_pa_follow_ori_ang_z.control(shared.pose_diff_ctrl_pa_follow_ori.rot[2]);
    // ctrl_pa_follow_ori_ang_y
    shared.eacc_ctrl_pa_follow_ori_ang_y = state.ctrl_pa_follow_ori_ang_y.control(shared.pose_diff_ctrl_pa_follow_ori.rot[1]);
    // ctrl_pa_follow_ori_ang_x
    shared.eacc_ctrl_pa_follow_ori_ang_x = state.ctrl_pa_follow_ori_ang_x.control(shared.pose_diff_ctrl_pa_follow_ori.rot[0]);
    // ctrl_pa_follow_pos_lin_z
    shared.eacc_ctrl_pa_follow_pos_lin_z = state.ctrl_pa_follow_pos_lin_z.control(shared.pose_diff_ctrl_pa_follow_pos.vel[2]);
    // ctrl_pa_follow_pos_lin_y
    shared.eacc_ctrl_pa_follow_pos_lin_y = state.ctrl_pa_follow_pos_lin_y.control(shared.pose_diff_ctrl_pa_follow_pos.vel[1]);
    // ctrl_pa_follow_pos_lin_x
    shared.eacc_ctrl_pa_follow_pos_lin_x = state.ctrl_pa_follow_pos_lin_x.control(shared.pose_diff_ctrl_pa_follow_pos.vel[0]);



    KDL::SetToZero(state.arm_solver.f_cstr);
    state.arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 0) = 1.0;
    state.arm_solver.e_acc(0) = motion_spec::runtime::clamp_abs(shared.eacc_ctrl_pa_follow_pos_lin_x, motion_spec::runtime::kBetaMaxLin);

    state.arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 1) = 1.0;
    state.arm_solver.e_acc(1) = motion_spec::runtime::clamp_abs(shared.eacc_ctrl_pa_follow_pos_lin_y, motion_spec::runtime::kBetaMaxLin);

    state.arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 2) = 1.0;
    state.arm_solver.e_acc(2) = motion_spec::runtime::clamp_abs(shared.eacc_ctrl_pa_follow_pos_lin_z, motion_spec::runtime::kBetaMaxLin);

    state.arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::X), 3) = 1.0;
    state.arm_solver.e_acc(3) = motion_spec::runtime::clamp_abs(shared.eacc_ctrl_pa_follow_ori_ang_x, motion_spec::runtime::kBetaMaxRot);

    state.arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Y), 4) = 1.0;
    state.arm_solver.e_acc(4) = motion_spec::runtime::clamp_abs(shared.eacc_ctrl_pa_follow_ori_ang_y, motion_spec::runtime::kBetaMaxRot);

    state.arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Z), 5) = 1.0;
    state.arm_solver.e_acc(5) = motion_spec::runtime::clamp_abs(shared.eacc_ctrl_pa_follow_ori_ang_z, motion_spec::runtime::kBetaMaxRot);
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

inline void apply_motion_pick_above(
    motion_pick_above_state &state,
    shared_data &shared,
    const robot_io &robot) {
    for (int i = 0; i < state.arm_solver.num_joints; ++i) {
        robot.arm_solver.robot->jnt_trq_cmd[i] = motion_spec::runtime::clamp_abs(state.arm_solver.tau_ctrl(i), motion_spec::runtime::kTauMax);
    }
    mj_kdl::update(robot.arm_solver.robot);
    if (!mj_kdl::step(robot.arm_solver.robot)) {
        std::exit(0);
    }

}
