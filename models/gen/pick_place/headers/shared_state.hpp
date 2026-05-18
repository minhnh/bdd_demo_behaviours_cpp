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
    KDL::Vector direction_ctrl_rt_support_z;
    KDL::Vector direction_ctrl_og_support_z;
    KDL::Vector position_force_ctrl_pla_support_z = KDL::Vector(0.0, 0.0, 0.0);
    KDL::Vector position_force_ctrl_rt_support_z = KDL::Vector(0.0, 0.0, 0.0);
    KDL::Vector cube_position = KDL::Vector(0.5, 0.0, 0.745);
    KDL::Vector table_position = KDL::Vector(0.0, 0.0, 0.72);
    KDL::Vector position_force_ctrl_og_support_z = KDL::Vector(0.0, 0.0, 0.0);
    KDL::Vector position_force_ctrl_pl_support_z = KDL::Vector(0.0, 0.0, 0.0);
    KDL::Vector robot_position = KDL::Vector(0.0, 0.0, 0.72);
    KDL::Frame pose_ee_wrt_cube;
    KDL::Frame pose_cube_g_pinch;
    KDL::Frame inverse_pose_ee_base;
    KDL::Frame pose_elbow_base;
    KDL::Frame pose_ee_base;
    KDL::Frame pose_cube_base;
    KDL::Twist twist_ee_base;
    KDL::Twist pose_diff_ctrl_hold_position;
    KDL::Twist pose_diff_ctrl_hold_orientation;
    KDL::Wrench wrench_force_ctrl_og_support_z;
    KDL::Wrench wrench_force_ctrl_pla_support_z;
    KDL::Wrench wrench_force_ctrl_rt_support_z;
    KDL::Wrench wrench_force_ctrl_pl_support_z;
    double pose_ee_base_distance_z_err_retreat;
    double pose_ee_wrt_cube_distance_z_err_lift;
    double eacc_ctrl_hold_orientation_ang_z;
    double pose_ee_wrt_cube_rotation_y_err_grasp_hold;
    double pose_elbow_base_distance_z_err_grasp_hold;
    double eacc_pose_ee_base_distance_x_place_above;
    double pose_ee_wrt_cube_distance_x_err_pick;
    KDL::Frame home_pose;
    double grasp_pitch = 0.0;
    double eacc_ctrl_hold_orientation_ang_y;
    double eacc_pose_ee_wrt_cube_distance_y_pick_above;
    double pose_ee_wrt_cube_rotation_x_err_lift;
    double pose_ee_wrt_cube_distance_z_err_pick;
    double pose_ee_wrt_cube_rotation_y_err_lift;
    double eacc_ctrl_hold_orientation_ang_x;
    double eacc_pose_ee_base_distance_z_place_above;
    double pose_ee_wrt_cube_rotation_x_err_grasp_hold;
    double eacc_pose_ee_wrt_cube_rotation_x_pick;
    double pose_ee_base_distance_y_err_retreat;
    double pose_ee_wrt_cube_rotation_y_err_place_above;
    double pose_ee_wrt_cube_rotation_y_err_retreat;
    double eacc_pose_ee_wrt_cube_rotation_x_pick_above;
    double pose_ee_base_distance_y_err_place_above;
    double pose_elbow_base_distance_z_err_retreat;
    double twist_ee_base_linear_z_err;
    double eacc_ctrl_hold_position_lin_y;
    double eacc_pose_ee_wrt_cube_distance_z_lift;
    double force_ctrl_pl_support_z;
    double eacc_pose_ee_wrt_cube_rotation_z_pick_above;
    double pose_ee_wrt_cube_rotation_z_err_pick;
    double eacc_pose_ee_base_distance_x_place;
    double lift_z = 0.32;
    double eacc_pose_ee_wrt_cube_distance_x_pick;
    double pose_ee_base_distance_y_err_place;
    double place_z = 0.02;
    double pose_cube_g_pinch_distance_err;
    double eacc_pose_ee_wrt_cube_rotation_y_pick;
    double pose_ee_wrt_cube_rotation_x_err_place_above;
    double eacc_pose_ee_wrt_cube_rotation_y_place;
    double pose_ee_wrt_cube_rotation_z_err_pick_above;
    double pose_ee_base_distance_x_err_place_above;
    double eacc_pose_ee_wrt_cube_distance_z_grasp_hold;
    double eacc_pose_ee_wrt_cube_rotation_z_place;
    double eacc_pose_ee_wrt_cube_rotation_y_lift;
    double eacc_pose_ee_base_distance_z_place;
    double grasp_reach = 0.02;
    double eacc_pose_ee_wrt_cube_rotation_x_lift;
    double force_ctrl_pla_support_z;
    double eacc_pose_ee_wrt_cube_distance_x_lift;
    double pose_ee_wrt_cube_rotation_x_err_pick;
    double eacc_pose_ee_wrt_cube_rotation_z_lift;
    double pose_ee_wrt_cube_rotation_z_err_place_above;
    double pose_ee_wrt_cube_distance_x_err_grasp_hold;
    double eacc_pose_ee_wrt_cube_rotation_z_grasp_hold;
    double pose_ee_base_orientation;
    double force_ctrl_rt_support_z;
    double pose_ee_wrt_cube_distance_z_err_pick_above;
    double eacc_pose_ee_wrt_cube_distance_z_pick;
    double pose_ee_wrt_cube_rotation_x_err_retreat;
    double eacc_pose_ee_wrt_cube_rotation_x_retreat;
    double pose_ee_wrt_cube_distance_x_err_pick_above;
    double support_z;
    double eacc_pose_ee_wrt_cube_rotation_x_place_above;
    double pose_ee_wrt_cube_distance_z_err_grasp_hold;
    double force_ctrl_og_support_z;
    double eacc_ctrl_hold_position_lin_z;
    double eacc_pose_ee_wrt_cube_rotation_x_grasp_hold;
    double pose_ee_wrt_cube_distance_y_err_pick;
    double pose_ee_base_position;
    double eacc_pose_ee_base_distance_y_retreat;
    double eacc_pose_ee_wrt_cube_distance_y_pick;
    double eacc_pose_ee_wrt_cube_rotation_z_place_above;
    double pose_ee_wrt_cube_distance_y_err_lift;
    double zero_linvel = 0.0;
    double pose_ee_wrt_cube_rotation_z_err_grasp_hold;
    double eacc_pose_ee_wrt_cube_distance_y_lift;
    double pose_ee_wrt_cube_rotation_y_err_pick_above;
    double gravity_vec;
    double pose_elbow_base_distance_z_err_place;
    double eacc_pose_ee_base_distance_x_retreat;
    double pose_cube_g_pinch_distance;
    double pose_ee_base_distance_z_err_place;
    double pose_ee_wrt_cube_distance_x_err_lift;
    double eacc_pose_ee_wrt_cube_rotation_y_retreat;
    double grasp_yaw = 0.0;
    double grasp_z = 0.02;
    double eacc_pose_ee_base_distance_z_retreat;
    double eacc_pose_ee_wrt_cube_distance_x_grasp_hold;
    double place_x = 0.4;
    double pose_elbow_base_distance_z_err_place_above;
    double pose_ee_base_distance_z_err_place_above;
    double eacc_pose_ee_wrt_cube_rotation_z_pick;
    double pose_ee_wrt_cube_rotation_y_err_place;
    double grasp_roll = 3.14159;
    double place_y = 0.24;
    double eacc_pose_ee_wrt_cube_rotation_z_retreat;
    double eacc_pose_ee_wrt_cube_distance_x_pick_above;
    double pose_ee_wrt_cube_distance_y_err_pick_above;
    double place_above_z = 0.22;
    double eacc_pose_ee_wrt_cube_rotation_y_place_above;
    double pose_ee_wrt_cube_rotation_z_err_retreat;
    double pose_ee_wrt_cube_rotation_z_err_lift;
    double pose_ee_base_distance_x_err_retreat;
    double eacc_pose_ee_wrt_cube_distance_y_grasp_hold;
    double pose_ee_base_distance_x_err_place;
    double eacc_pose_ee_wrt_cube_rotation_x_place;
    double eacc_pose_ee_base_distance_y_place_above;
    double pose_ee_wrt_cube_rotation_y_err_pick;
    double above_z = 0.22;
    double eacc_pose_ee_wrt_cube_rotation_y_grasp_hold;
    double eacc_pose_ee_wrt_cube_distance_z_pick_above;
    double pose_ee_wrt_cube_rotation_z_err_place;
    double pose_ee_wrt_cube_distance_y_err_grasp_hold;
    double zero_dist = 0.0;
    double pose_ee_wrt_cube_rotation_x_err_place;
    double pose_ee_wrt_cube_rotation_x_err_pick_above;
    double eacc_ctrl_hold_position_lin_x;
    double eacc_pose_ee_base_distance_y_place;
    double eacc_pose_ee_wrt_cube_rotation_y_pick_above;
};
