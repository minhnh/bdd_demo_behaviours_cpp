#pragma once

#include "runtime.hpp"
#include "shared_state.hpp"

struct motion_place_above_state {
    bool active = false;
    arm_solver_solver_state arm_solver;
    bool snapshot_taken = false;
    motion_spec::runtime::PIDControl ctrl_pla_reach_x{100.0, 10.0, 20.0, 0.9};
    motion_spec::runtime::PIDControl ctrl_pla_reach_y{100.0, 10.0, 20.0, 0.9};
    motion_spec::runtime::PIDControl ctrl_pla_reach_z{100.0, 10.0, 20.0, 0.9};
    motion_spec::runtime::PIDControl ctrl_pla_align_roll{80.0, 5.0, 40.0, 0.9};
    motion_spec::runtime::PIDControl ctrl_pla_align_pitch{80.0, 5.0, 40.0, 0.9};
    motion_spec::runtime::PIDControl ctrl_pla_align_yaw{80.0, 5.0, 40.0, 0.9};
    motion_spec::runtime::ImpedanceControl ctrl_pla_support_z{800.0, 80.0};
    bool mon_place_above_settled_previous = false;
};

inline void reset_motion_place_above(motion_place_above_state &state) {
    state = motion_place_above_state{};
}

inline void init_motion_place_above(motion_place_above_state &state, const robot_io &robot) {
    if (!state.arm_solver.initialized) {
        state.arm_solver.num_constraints = 6;
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
        state.arm_solver.root_acc.vel = KDL::Vector(-0.0, -0.0, 9.81);
        state.arm_solver.achd_fext = std::make_unique<KDL::ChainHdSolver_Vereshchagin_Fext_FixedJoint>(*robot.arm_solver.chain, state.arm_solver.root_acc, state.arm_solver.num_constraints);
        state.arm_solver.achd_acc = std::make_unique<KDL::ChainHdSolver_Vereshchagin_Fixed_Joint>(*robot.arm_solver.chain, state.arm_solver.root_acc, state.arm_solver.num_constraints);
        state.arm_solver.rnea = std::make_unique<KDL::ChainIdSolver_RNE>(*robot.arm_solver.chain, KDL::Vector());
        state.arm_solver.initialized = true;
    }
}

inline void update_motion_place_above(
    motion_place_above_state &state,
    shared_data &shared,
    const robot_io &robot) {
    init_motion_place_above(state, robot);

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
        shared.support_z = shared.pose_elbow_base.p[2];
        state.snapshot_taken = true;
    }
    shared.pose_ee_wrt_cube = shared.pose_cube_base.Inverse() * shared.pose_ee_base;

}

inline bool can_start_motion_place_above(
    motion_place_above_state &state,
    shared_data &shared) {
    return true;
}

inline void monitor_motion_place_above(
    motion_place_above_state &state,
    shared_data &shared) {
    // eval_place_above_until_lin_z_settled
    shared.twist_ee_base_linear_z_err = motion_spec::runtime::evaluate_equality_constraint(shared.zero_linvel, shared.twist_ee_base.vel[2]);

    {
        const bool active = motion_spec::runtime::constraint_satisfied(shared.twist_ee_base_linear_z_err);
        if (motion_spec::runtime::rising_edge(state.mon_place_above_settled_previous, active)) {
            motion_spec::runtime::warn_produce_event_not_implemented("place_above_settled");
        }
    }

}

inline void control_motion_place_above(
    motion_place_above_state &state,
    shared_data &shared,
    const robot_io &robot) {
    {
        KDL::Frame _pose_axis_target_pose_axis_error_pose_ee_base = shared.pose_ee_base;
        _pose_axis_target_pose_axis_error_pose_ee_base.p[0] = shared.place_x;
        _pose_axis_target_pose_axis_error_pose_ee_base.p[1] = shared.place_y;
        _pose_axis_target_pose_axis_error_pose_ee_base.p[2] = shared.place_above_z;
        const double _pose_axis_roll_pose_axis_error_pose_ee_base = 0.0;
        const double _pose_axis_pitch_pose_axis_error_pose_ee_base = 0.0;
        const double _pose_axis_yaw_pose_axis_error_pose_ee_base = 0.0;
        const KDL::Twist _pose_axis_error_pose_axis_error_pose_ee_base = KDL::diff(shared.pose_ee_base, _pose_axis_target_pose_axis_error_pose_ee_base);
        const double _pose_axis_error_linear_X = _pose_axis_error_pose_axis_error_pose_ee_base.vel.x();
        const double _pose_axis_error_linear_Y = _pose_axis_error_pose_axis_error_pose_ee_base.vel.y();
        const double _pose_axis_error_linear_Z = _pose_axis_error_pose_axis_error_pose_ee_base.vel.z();
        const double _pose_axis_error_angular_X = _pose_axis_error_pose_axis_error_pose_ee_base.rot.x();
        const double _pose_axis_error_angular_Y = _pose_axis_error_pose_axis_error_pose_ee_base.rot.y();
        const double _pose_axis_error_angular_Z = _pose_axis_error_pose_axis_error_pose_ee_base.rot.z();
        shared.pose_ee_base_distance_x_err_place_above = _pose_axis_error_linear_X;
        shared.pose_ee_base_distance_y_err_place_above = _pose_axis_error_linear_Y;
        shared.pose_ee_base_distance_z_err_place_above = _pose_axis_error_linear_Z;
    }
    {
        KDL::Frame _pose_axis_target_pose_axis_error_pose_ee_wrt_cube = shared.pose_ee_wrt_cube;
        const double _pose_axis_roll_pose_axis_error_pose_ee_wrt_cube = shared.grasp_roll;
        const double _pose_axis_pitch_pose_axis_error_pose_ee_wrt_cube = shared.grasp_pitch;
        const double _pose_axis_yaw_pose_axis_error_pose_ee_wrt_cube = shared.grasp_yaw;
        _pose_axis_target_pose_axis_error_pose_ee_wrt_cube.M = KDL::Rotation::RPY(_pose_axis_roll_pose_axis_error_pose_ee_wrt_cube, _pose_axis_pitch_pose_axis_error_pose_ee_wrt_cube, _pose_axis_yaw_pose_axis_error_pose_ee_wrt_cube);
        const KDL::Twist _pose_axis_error_pose_axis_error_pose_ee_wrt_cube = KDL::diff(shared.pose_ee_wrt_cube, _pose_axis_target_pose_axis_error_pose_ee_wrt_cube);
        const double _pose_axis_error_linear_X = _pose_axis_error_pose_axis_error_pose_ee_wrt_cube.vel.x();
        const double _pose_axis_error_linear_Y = _pose_axis_error_pose_axis_error_pose_ee_wrt_cube.vel.y();
        const double _pose_axis_error_linear_Z = _pose_axis_error_pose_axis_error_pose_ee_wrt_cube.vel.z();
        const double _pose_axis_error_angular_X = _pose_axis_error_pose_axis_error_pose_ee_wrt_cube.rot.x();
        const double _pose_axis_error_angular_Y = _pose_axis_error_pose_axis_error_pose_ee_wrt_cube.rot.y();
        const double _pose_axis_error_angular_Z = _pose_axis_error_pose_axis_error_pose_ee_wrt_cube.rot.z();
        shared.pose_ee_wrt_cube_rotation_x_err_place_above = _pose_axis_error_angular_X;
        shared.pose_ee_wrt_cube_rotation_y_err_place_above = _pose_axis_error_angular_Y;
        shared.pose_ee_wrt_cube_rotation_z_err_place_above = _pose_axis_error_angular_Z;
    }
    // eval_place_above_while_support_elbow_z
    shared.pose_elbow_base_distance_z_err_place_above = motion_spec::runtime::evaluate_equality_constraint(shared.support_z, shared.pose_elbow_base.p[2]);
    // ctrl_pla_support_z
    shared.force_ctrl_pla_support_z = state.ctrl_pla_support_z.control(shared.pose_elbow_base_distance_z_err_place_above);
    // ctrl_pla_align_yaw
    shared.eacc_pose_ee_wrt_cube_rotation_z_place_above = state.ctrl_pla_align_yaw.control(shared.pose_ee_wrt_cube_rotation_z_err_place_above);
    // ctrl_pla_align_pitch
    shared.eacc_pose_ee_wrt_cube_rotation_y_place_above = state.ctrl_pla_align_pitch.control(shared.pose_ee_wrt_cube_rotation_y_err_place_above);
    // ctrl_pla_align_roll
    shared.eacc_pose_ee_wrt_cube_rotation_x_place_above = state.ctrl_pla_align_roll.control(shared.pose_ee_wrt_cube_rotation_x_err_place_above);
    // ctrl_pla_reach_z
    shared.eacc_pose_ee_base_distance_z_place_above = state.ctrl_pla_reach_z.control(shared.pose_ee_base_distance_z_err_place_above);
    // ctrl_pla_reach_y
    shared.eacc_pose_ee_base_distance_y_place_above = state.ctrl_pla_reach_y.control(shared.pose_ee_base_distance_y_err_place_above);
    // ctrl_pla_reach_x
    shared.eacc_pose_ee_base_distance_x_place_above = state.ctrl_pla_reach_x.control(shared.pose_ee_base_distance_x_err_place_above);
    // compute_direction_ctrl_pla_support_z
    shared.direction_ctrl_pla_support_z = shared.pose_elbow_base.p;
    shared.direction_ctrl_pla_support_z.Normalize();
    // compute_wrench_force_ctrl_pla_support_z
    shared.wrench_force_ctrl_pla_support_z = KDL::Wrench(shared.direction_ctrl_pla_support_z * shared.force_ctrl_pla_support_z, KDL::Vector(0.0, 0.0, 0.0)).RefPoint(-shared.position_force_ctrl_pla_support_z);



    KDL::SetToZero(state.arm_solver.f_cstr);
    state.arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::X), 0) = 1.0;
    state.arm_solver.e_acc(0) = shared.eacc_pose_ee_base_distance_x_place_above;

    state.arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Y), 1) = 1.0;
    state.arm_solver.e_acc(1) = shared.eacc_pose_ee_base_distance_y_place_above;

    state.arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Linear, motion_spec::runtime::Axis::Z), 2) = 1.0;
    state.arm_solver.e_acc(2) = shared.eacc_pose_ee_base_distance_z_place_above;

    state.arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::X), 3) = 1.0;
    state.arm_solver.e_acc(3) = shared.eacc_pose_ee_wrt_cube_rotation_x_place_above;

    state.arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Y), 4) = 1.0;
    state.arm_solver.e_acc(4) = shared.eacc_pose_ee_wrt_cube_rotation_y_place_above;

    state.arm_solver.f_cstr(motion_spec::runtime::constraint_row(motion_spec::runtime::Subspace::Angular, motion_spec::runtime::Axis::Z), 5) = 1.0;
    state.arm_solver.e_acc(5) = shared.eacc_pose_ee_wrt_cube_rotation_z_place_above;
    KDL::SetToZero(state.arm_solver.tau_ff);
    for (int i = 0; i < state.arm_solver.num_segments; ++i) {
        KDL::SetToZero(state.arm_solver.f_ext[i]);
    }
    state.arm_solver.f_ext[motion_spec::runtime::find_segment_index(*robot.arm_solver.chain, "half_arm_2_link") - 1] += shared.wrench_force_ctrl_pla_support_z;
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

inline void apply_motion_place_above(
    motion_place_above_state &state,
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
