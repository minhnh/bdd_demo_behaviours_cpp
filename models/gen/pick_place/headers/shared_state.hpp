#pragma once

#include "runtime.hpp"

#include <memory>
#include <kdl/frames.hpp>
#include <kdl/chain.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainhdsolver_vereshchagin_fixed_joint.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <mj_kdl_wrapper/mj_kdl_wrapper.hpp>
#include <kdl/chainhdsolver_vereshchagin_fext_fixed_joint.hpp>

inline constexpr int KINOVA_NUM_JOINTS = 7;



struct manipulator_robot {
    mj_kdl::Robot *robot = nullptr;
    KDL::Chain *chain = nullptr;
};

struct arm_solver_solver_state {
    bool initialized = false;
    int num_constraints = 0;
    int num_joints = 0;
    int num_segments = 0;
    KDL::Twist root_acc;
    KDL::JntArray q;
    KDL::JntArray qd;
    KDL::JntArray qdd;
    KDL::JntArray tau_ff;
    KDL::JntArray tau_ctrl;
    KDL::Wrenches f_ext;
    KDL::Jacobian f_cstr;
    KDL::JntArray e_acc;
    std::unique_ptr<KDL::ChainHdSolver_Vereshchagin_Fext_FixedJoint> achd_fext;
    std::unique_ptr<KDL::ChainHdSolver_Vereshchagin_Fixed_Joint> achd_acc;
    std::unique_ptr<KDL::ChainIdSolver_RNE> rnea;
};

struct robot_io {
    manipulator_robot arm_solver;
};

struct shared_data {
    KDL::Vector direction_ctrl_pla_support_z;
    KDL::Vector direction_ctrl_pl_support_z;
    KDL::Vector direction_ctrl_og_support_z;
    KDL::Vector direction_ctrl_rt_support_z;
    KDL::Vector robot_position = KDL::Vector(0.0, 0.0, 0.72);
    KDL::Vector cube_position = KDL::Vector(0.5, 0.0, 0.745);
    KDL::Vector position_force_ctrl_pla_support_z = KDL::Vector(0.0, 0.0, 0.0);
    KDL::Vector position_force_ctrl_rt_support_z = KDL::Vector(0.0, 0.0, 0.0);
    KDL::Vector position_force_ctrl_og_support_z = KDL::Vector(0.0, 0.0, 0.0);
    KDL::Vector position_force_ctrl_pl_support_z = KDL::Vector(0.0, 0.0, 0.0);
    KDL::Vector table_position = KDL::Vector(0.0, 0.0, 0.72);
    KDL::Frame pose_cube_g_pinch;
    KDL::Frame pose_cube_base;
    KDL::Frame pose_ee_base;
    KDL::Frame goal_pose;
    KDL::Frame pose_elbow_base;
    KDL::Frame inverse_pose_ee_base;
    KDL::Twist twist_ee_base;
    KDL::Twist pose_diff_ctrl_lt_hold_orientation;
    KDL::Twist pose_diff_ctrl_hold_position;
    KDL::Twist pose_diff_ctrl_rt_hold_orientation;
    KDL::Twist pose_diff_ctrl_hold_orientation;
    KDL::Twist pose_diff_ctrl_pl_hold_orientation;
    KDL::Twist pose_diff_ctrl_og_hold_orientation;
    KDL::Twist pose_diff_ctrl_pla_hold_orientation;
    KDL::Twist pose_diff_ctrl_pa_follow_ori;
    KDL::Twist pose_diff_ctrl_pk_hold_orientation;
    KDL::Twist pose_diff_ctrl_pa_follow_pos;
    KDL::Twist pose_diff_ctrl_cg_hold_orientation;
    KDL::Wrench wrench_force_ctrl_pl_support_z;
    KDL::Wrench wrench_force_ctrl_og_support_z;
    KDL::Wrench wrench_force_ctrl_rt_support_z;
    KDL::Wrench wrench_force_ctrl_pla_support_z;
    double eacc_ctrl_lt_hold_orientation_ang_x = 0.0;
    double ctrl_pa_follow_ori_err_ang_z = 0.0;
    double pose_ee_base_distance_y_err_retreat = 0.0;
    double pose_ee_base_distance_x_err_place = 0.0;
    double ctrl_pla_hold_orientation_err_ang_y = 0.0;
    double pose_ee_base_distance_x_err_lift = 0.0;
    KDL::Frame place_orientation_pose;
    double ctrl_pk_hold_orientation_err_ang_y = 0.0;
    double eacc_ctrl_cg_hold_orientation_ang_y = 0.0;
    double eacc_pose_ee_base_distance_y_pick = 0.0;
    double ctrl_lt_hold_orientation_err_ang_y = 0.0;
    double grasp_y = 0.0;
    double ctrl_pa_follow_pos_err_lin_x = 0.0;
    double eacc_pose_ee_base_distance_y_place = 0.0;
    double hold_x = 0.0;
    double eacc_pose_ee_base_distance_x_lift = 0.0;
    KDL::Frame home_pose;
    double ctrl_pk_hold_orientation_err_ang_x = 0.0;
    double force_ctrl_pla_support_z = 0.0;
    double ctrl_hold_position_err_lin_z = 0.0;
    double pose_cube_g_pinch_distance_err = 0.0;
    double eacc_ctrl_hold_orientation_ang_x = 0.0;
    double ctrl_cg_hold_orientation_err_ang_z = 0.0;
    double force_ctrl_pl_support_z = 0.0;
    double place_z = 0.075;
    double ctrl_pa_follow_pos_err_lin_z = 0.0;
    double eacc_pose_ee_base_distance_z_place_above = 0.0;
    double eacc_pose_ee_base_distance_z_place = 0.0;
    double place_y = 0.24;
    double eacc_ctrl_og_hold_orientation_ang_y = 0.0;
    double pose_ee_base_distance_y_err_place = 0.0;
    double eacc_ctrl_pk_hold_orientation_ang_x = 0.0;
    KDL::Frame retreat_orientation_pose;
    double zero_dist = 0.0;
    double ctrl_cg_hold_orientation_err_ang_y = 0.0;
    double pose_elbow_base_distance_z = 0.0;
    double pose_elbow_base_distance_z_err_place = 0.0;
    double eacc_pose_ee_base_distance_y_lift = 0.0;
    double grasp_yaw = 1.5708;
    double pose_ee_base_distance_y_err_place_above = 0.0;
    double pose_ee_base_distance_x_err = 0.0;
    double lift_x = 0.0;
    double above_z = 0.26;
    double pose_ee_base_distance_y = 0.0;
    double ctrl_pa_follow_ori_err_ang_x = 0.0;
    double start_cube_x = 0.0;
    double pose_ee_base_distance_y_err_lift = 0.0;
    double eacc_ctrl_hold_position_lin_y = 0.0;
    double eacc_pose_ee_base_distance_x_retreat = 0.0;
    double hold_z = 0.0;
    double eacc_ctrl_pk_hold_orientation_ang_z = 0.0;
    KDL::Frame pose_ee_base_pose;
    double support_z = 0.0;
    KDL::Frame traj;
    double grasp_x = 0.0;
    KDL::Frame lift_orientation_pose;
    KDL::Frame hold_orientation_pose;
    double eacc_ctrl_rt_hold_orientation_ang_z = 0.0;
    double eacc_ctrl_pla_hold_orientation_ang_x = 0.0;
    double ctrl_og_hold_orientation_err_ang_z = 0.0;
    double pose_ee_base_distance_y_err = 0.0;
    double eacc_ctrl_pl_hold_orientation_ang_z = 0.0;
    double ctrl_pla_hold_orientation_err_ang_x = 0.0;
    KDL::Frame start_pose;
    double pose_elbow_base_distance_z_err_place_above = 0.0;
    double pose_ee_base_distance_x_err_grasp_hold = 0.0;
    double ctrl_lt_hold_orientation_err_ang_x = 0.0;
    double pose_ee_base_distance_y_err_pick = 0.0;
    double lift_y = 0.0;
    double eacc_pose_ee_base_distance_z_pick = 0.0;
    double gravity_vec = 0.0;
    KDL::Frame grasp_orientation;
    double eacc_ctrl_pa_follow_pos_lin_y = 0.0;
    double grasp_pitch = 0.0;
    double ctrl_pl_hold_orientation_err_ang_z = 0.0;
    double place_above_z = 0.4;
    double eacc_pose_ee_base_distance_x_place = 0.0;
    double eacc_ctrl_pa_follow_ori_ang_z = 0.0;
    double goal_pose_position = 0.0;
    double ctrl_hold_orientation_err_ang_y = 0.0;
    double ctrl_hold_position_err_lin_y = 0.0;
    double pose_ee_base_distance_z_err_retreat = 0.0;
    double force_ctrl_rt_support_z = 0.0;
    double ctrl_og_hold_orientation_err_ang_x = 0.0;
    KDL::Frame place_above_orientation_pose;
    double ctrl_pl_hold_orientation_err_ang_x = 0.0;
    double start_cube_y = 0.0;
    double ctrl_pl_hold_orientation_err_ang_y = 0.0;
    double eacc_pose_ee_base_distance_y_retreat = 0.0;
    double ctrl_rt_hold_orientation_err_ang_z = 0.0;
    double pose_ee_base_distance_z_err_place = 0.0;
    double grasp_z = 0.05;
    double ctrl_hold_orientation_err_ang_z = 0.0;
    double ctrl_og_hold_orientation_err_ang_y = 0.0;
    double eacc_pose_ee_base_distance_z_lift = 0.0;
    double pose_ee_base_distance_x_err_place_above = 0.0;
    double eacc_ctrl_hold_position_lin_z = 0.0;
    double grasp_roll = 3.14159;
    double pose_elbow_base_distance_z_err_retreat = 0.0;
    double pose_ee_base_distance_z_err_lift = 0.0;
    double ctrl_cg_hold_orientation_err_ang_x = 0.0;
    double eacc_ctrl_rt_hold_orientation_ang_x = 0.0;
    double eacc_ctrl_hold_position_lin_x = 0.0;
    double hold_y = 0.0;
    double eacc_ctrl_cg_hold_orientation_ang_x = 0.0;
    double force_ctrl_og_support_z = 0.0;
    double pose_ee_base_distance_x_err_retreat = 0.0;
    double eacc_ctrl_pa_follow_pos_lin_x = 0.0;
    double pose_ee_base_distance_z = 0.0;
    double ctrl_pa_follow_pos_err_lin_y = 0.0;
    double place_above_up_z = 0.2;
    double ctrl_hold_position_err_lin_x = 0.0;
    double eacc_ctrl_pa_follow_ori_ang_y = 0.0;
    double eacc_ctrl_rt_hold_orientation_ang_y = 0.0;
    double lift_z_up = 0.2;
    double eacc_ctrl_cg_hold_orientation_ang_z = 0.0;
    double pose_ee_base_distance_x_err_pick = 0.0;
    double ctrl_pla_hold_orientation_err_ang_z = 0.0;
    double place_x = 0.4;
    double eacc_ctrl_pa_follow_pos_lin_z = 0.0;
    double ctrl_rt_hold_orientation_err_ang_y = 0.0;
    double eacc_ctrl_pl_hold_orientation_ang_y = 0.0;
    double eacc_ctrl_og_hold_orientation_ang_x = 0.0;
    double pose_ee_base_distance_y_err_grasp_hold = 0.0;
    double zero_linvel = 0.0;
    double ctrl_hold_orientation_err_ang_x = 0.0;
    double eacc_ctrl_pl_hold_orientation_ang_x = 0.0;
    double pose_ee_base_distance_x = 0.0;
    double pose_elbow_base_distance_z_err_grasp_hold = 0.0;
    double ctrl_pa_follow_ori_err_ang_y = 0.0;
    double alpha = 0.0;
    double pose_ee_base_distance_z_err_place_above = 0.0;
    double eacc_ctrl_pla_hold_orientation_ang_z = 0.0;
    double eacc_ctrl_lt_hold_orientation_ang_y = 0.0;
    double eacc_pose_ee_base_distance_z_retreat = 0.0;
    double eacc_ctrl_hold_orientation_ang_y = 0.0;
    double lift_z = 0.4;
    double pose_ee_base_orientation = 0.0;
    double eacc_pose_ee_base_distance_y_place_above = 0.0;
    double ctrl_rt_hold_orientation_err_ang_x = 0.0;
    double ctrl_pk_hold_orientation_err_ang_z = 0.0;
    double pose_ee_base_distance_z_err = 0.0;
    double eacc_ctrl_lt_hold_orientation_ang_z = 0.0;
    double eacc_ctrl_hold_orientation_ang_z = 0.0;
    double eacc_ctrl_pk_hold_orientation_ang_y = 0.0;
    double goal_pose_orientation = 0.0;
    double eacc_pose_ee_base_distance_z_grasp_hold = 0.0;
    double eacc_pose_ee_base_distance_y_grasp_hold = 0.0;
    double eacc_pose_ee_base_distance_x_pick = 0.0;
    double ctrl_lt_hold_orientation_err_ang_z = 0.0;
    double pose_ee_base_distance_z_err_pick = 0.0;
    double eacc_ctrl_og_hold_orientation_ang_z = 0.0;
    double eacc_ctrl_pa_follow_ori_ang_x = 0.0;
    double pose_ee_base_position = 0.0;
    double pose_cube_g_pinch_distance = 0.0;
    double pose_ee_base_distance_z_err_grasp_hold = 0.0;
    double eacc_pose_ee_base_distance_x_place_above = 0.0;
    double eacc_pose_ee_base_distance_x_grasp_hold = 0.0;
    KDL::Frame pose_ee_base_pose_err;
    double eacc_ctrl_pla_hold_orientation_ang_y = 0.0;
    double grasp_reach = 0.05;
};
