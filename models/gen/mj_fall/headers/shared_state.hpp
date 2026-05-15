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
    KDL::Jacobian f_cstr;
    KDL::JntArray e_acc;
    std::unique_ptr<KDL::ChainHdSolver_Vereshchagin_Fixed_Joint> achd_acc;
    std::unique_ptr<KDL::ChainIdSolver_RNE> rnea;
};

struct robot_io {
    manipulator_robot arm_solver;
};

struct shared_data {
    KDL::Frame pose_ee;
    KDL::Twist twist_ee;
    KDL::Twist pose_diff_ctrl_orientation;
    double eacc_ctrl_orientation_ang_x;
    double eacc_twist_ee_linear_x_fall_down;
    double vel_y_zero = 0.0;
    double eacc_ctrl_orientation_ang_y;
    double vel_z_down = -0.01;
    double eacc_twist_ee_linear_z_fall_down;
    double eacc_ctrl_orientation_ang_z;
    double gravity_vec;
    double pose_ee_orientation;
    double twist_ee_linear_y_err_fall_down;
    double twist_ee_linear_x_err_fall_down;
    double vel_x_zero = 0.0;
    double twist_ee_linear_z_err_fall_down;
    double eacc_twist_ee_linear_y_fall_down;
    KDL::Frame pose_ee_start;
};
