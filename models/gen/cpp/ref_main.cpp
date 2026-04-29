#include "runtime.hpp"
#include "shared_state.hpp"
#include "motion_idle.hpp"
#include "motion_touch_down.hpp"
#include "motion_move_forward_15cm.hpp"
#include "motion_move_up_30cm_from_table.hpp"

#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>
#include <kdl_parser/kdl_parser.hpp>
#include <unistd.h>

int main() {
    robot_io robot{};
    shared_data shared{};

    urdf::ModelInterfaceSharedPtr kinova_model = urdf::parseURDFFile("../robots/kg3.urdf");
    KDL::Tree kinova_tree;
    kdl_parser::treeFromUrdfModel(*kinova_model, kinova_tree);
    kinova_state kinova_arm_solver_state{};
    kinova_arm_solver_state.ctrl_mode = ROBIF2B_CTRL_MODE_FORCE;

    robif2b_kinova_gen3_nbx kinova_arm_solver{};
    kinova_arm_solver.conf.ip_address = "127.0.0.1";
    kinova_arm_solver.conf.port = 10000;
    kinova_arm_solver.conf.port_real_time = 10001;
    kinova_arm_solver.conf.user = "admin";
    kinova_arm_solver.conf.password = "admin";
    kinova_arm_solver.conf.session_timeout = 60000;
    kinova_arm_solver.conf.connection_timeout = 2000;
    kinova_arm_solver.ctrl_mode = &kinova_arm_solver_state.ctrl_mode;
    kinova_arm_solver.jnt_pos_msr = &kinova_arm_solver_state.pos_msr[0];
    kinova_arm_solver.jnt_vel_msr = &kinova_arm_solver_state.vel_msr[0];
    kinova_arm_solver.jnt_trq_msr = &kinova_arm_solver_state.eff_msr[0];
    kinova_arm_solver.act_cur_msr = &kinova_arm_solver_state.cur_msr[0];
    kinova_arm_solver.jnt_pos_cmd = &kinova_arm_solver_state.pos_cmd[0];
    kinova_arm_solver.jnt_vel_cmd = &kinova_arm_solver_state.vel_cmd[0];
    kinova_arm_solver.jnt_trq_cmd = &kinova_arm_solver_state.eff_cmd[0];
    kinova_arm_solver.act_cur_cmd = &kinova_arm_solver_state.cur_cmd[0];
    kinova_arm_solver.success = &kinova_arm_solver_state.success;

    KDL::Chain chain_arm_solver;
    kinova_tree.getChain("link-base", "link-ee", chain_arm_solver);

    robot.arm_solver = manipulator_robot{
        .state = &kinova_arm_solver_state,
        .robot = &kinova_arm_solver,
        .chain = &chain_arm_solver,
    };

    motion_idle_state motion_idle_state_instance;
    motion_touch_down_state motion_touch_down_state_instance;
    motion_move_forward_15cm_state motion_move_forward_15cm_state_instance;
    motion_move_up_30cm_from_table_state motion_move_up_30cm_from_table_state_instance;
    robif2b_kinova_gen3_configure(&kinova_arm_solver);
    robif2b_kinova_gen3_recover(&kinova_arm_solver);
    robif2b_kinova_gen3_start(&kinova_arm_solver);


    int current_motion = 0;
    while (true) {
        switch (current_motion) {
        case 0: {
            update_motion_idle(motion_idle_state_instance, shared, robot);
            monitor_motion_idle(motion_idle_state_instance, shared);
            if (can_start_motion_idle(motion_idle_state_instance, shared)) {
                control_motion_idle(motion_idle_state_instance, shared, robot);
            }
            apply_motion_idle(motion_idle_state_instance, shared, robot);
            current_motion = 1;
            break;
        }

        case 1: {
            update_motion_touch_down(motion_touch_down_state_instance, shared, robot);
            monitor_motion_touch_down(motion_touch_down_state_instance, shared);
            if (can_start_motion_touch_down(motion_touch_down_state_instance, shared)) {
                control_motion_touch_down(motion_touch_down_state_instance, shared, robot);
            }
            apply_motion_touch_down(motion_touch_down_state_instance, shared, robot);
            if (motion_spec::runtime::constraint_satisfied(shared.twist_ee_base_linear_z_err)) {
                current_motion = 2;
            }
            break;
        }

        case 2: {
            update_motion_move_forward_15cm(motion_move_forward_15cm_state_instance, shared, robot);
            monitor_motion_move_forward_15cm(motion_move_forward_15cm_state_instance, shared);
            if (can_start_motion_move_forward_15cm(motion_move_forward_15cm_state_instance, shared)) {
                control_motion_move_forward_15cm(motion_move_forward_15cm_state_instance, shared, robot);
            }
            apply_motion_move_forward_15cm(motion_move_forward_15cm_state_instance, shared, robot);
            if (motion_spec::runtime::constraint_satisfied(shared.pose_start_ee_distance_x_err)) {
                current_motion = 3;
            }
            break;
        }

        case 3: {
            update_motion_move_up_30cm_from_table(motion_move_up_30cm_from_table_state_instance, shared, robot);
            monitor_motion_move_up_30cm_from_table(motion_move_up_30cm_from_table_state_instance, shared);
            if (can_start_motion_move_up_30cm_from_table(motion_move_up_30cm_from_table_state_instance, shared)) {
                control_motion_move_up_30cm_from_table(motion_move_up_30cm_from_table_state_instance, shared, robot);
            }
            apply_motion_move_up_30cm_from_table(motion_move_up_30cm_from_table_state_instance, shared, robot);
            if (motion_spec::runtime::constraint_satisfied(shared.pose_table_ee_distance_err)) {
                current_motion = 4;
            }
            break;
        }

        default:
            current_motion = 4 - 1;
            break;
        }
        usleep(1000);
    }

    robif2b_kinova_gen3_stop(&kinova_arm_solver);
    robif2b_kinova_gen3_shutdown(&kinova_arm_solver);

    return 0;
}