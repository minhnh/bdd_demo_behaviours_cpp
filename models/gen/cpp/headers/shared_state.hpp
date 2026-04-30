#pragma once

#include "runtime.hpp"

#include <memory>
#include <kdl/frames.hpp>
#include <kdl/chain.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl/chainhdsolver_vereshchagin.hpp>
#include <robif2b/functions/kinova_gen3.h>
#include "chainhdsolver_vereshchagin_fext.hpp"

inline constexpr int KINOVA_NUM_JOINTS = 7;

struct kinova_state {
    bool success = false;
    enum robif2b_ctrl_mode ctrl_mode = ROBIF2B_CTRL_MODE_FORCE;
    double pos_msr[KINOVA_NUM_JOINTS] = {0.0};
    double vel_msr[KINOVA_NUM_JOINTS] = {0.0};
    double eff_msr[KINOVA_NUM_JOINTS] = {0.0};
    double cur_msr[KINOVA_NUM_JOINTS] = {0.0};
    double pos_cmd[KINOVA_NUM_JOINTS] = {0.0};
    double vel_cmd[KINOVA_NUM_JOINTS] = {0.0};
    double eff_cmd[KINOVA_NUM_JOINTS] = {0.0};
    double cur_cmd[KINOVA_NUM_JOINTS] = {0.0};
};

struct manipulator_robot {
    kinova_state *state = nullptr;
    robif2b_kinova_gen3_nbx *robot = nullptr;
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
    KDL::Jacobian f_cstr;
    KDL::JntArray e_acc;
    std::unique_ptr<KDL::ChainHdSolver_Vereshchagin> achd_acc;
};

struct robot_io {
    manipulator_robot arm_solver;
};

struct shared_data {
    KDL::Frame pose_world_table;
    KDL::Frame pose_world_ee;
    KDL::Frame inverse_pose_world_table;
    KDL::Frame pose_table_ee;
    KDL::Frame pose_start_ee;
    KDL::Frame pose_ee_base;
    KDL::Twist twist_ee_base;
    KDL::Twist pose_diff_ctrl_hold_pose;
    double eacc_twist_ee_base_linear_z_touch_down;
    double zero_angvel = 0.0;
    double twist_ee_base_linear_z_err_touch_down;
    double zero_linvel = 0.0;
    KDL::Frame pose_ee_start;
    double eacc_twist_ee_base_linear_z_move_forward_15cm;
    double up_vel = 0.03;
    double eacc_twist_ee_base_linear_x_move_forward_15cm;
    double twist_ee_base_linear_z_err_move_up_30cm_from_table;
    double eacc_ctrl_hold_pose_ang_x;
    double eacc_twist_ee_base_linear_z_move_up_30cm_from_table;
    double eacc_ctrl_hold_pose_ang_z;
    double pose_start_ee_distance_x_err;
    double eacc_twist_ee_base_angular_z;
    double eacc_ctrl_hold_pose_ang_y;
    double eacc_ctrl_hold_pose_lin_z;
    double forward_distance = 0.15;
    double twist_ee_base_angular_y_err;
    double eacc_twist_ee_base_angular_x;
    double twist_ee_base_linear_z_err;
    double pose_table_ee_distance_err;
    double twist_ee_base_linear_y_err;
    double table_clearance = 0.3;
    double down_vel = -0.03;
    double twist_ee_base_linear_x_err;
    KDL::Frame pose_ee_base_pose;
    double eacc_twist_ee_base_linear_x;
    double eacc_ctrl_hold_pose_lin_y;
    double eacc_twist_ee_base_angular_y;
    double pose_table_ee_distance;
    double twist_ee_base_linear_x_err_move_forward_15cm;
    double eacc_ctrl_hold_pose_lin_x;
    double forward_vel = 0.05;
    double twist_ee_base_angular_x_err;
    double twist_ee_base_linear_z_err_move_forward_15cm;
    double eacc_twist_ee_base_linear_y;
    double twist_ee_base_angular_z_err;
    double gravity_vec;
};