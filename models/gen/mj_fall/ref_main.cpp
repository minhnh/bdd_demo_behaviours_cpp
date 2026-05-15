#include "runtime.hpp"
#include "shared_state.hpp"
#include "motion_fall_down.hpp"


#include <cstring>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

#include <unistd.h>

int main(int argc, char **argv) {
    robot_io robot{};
    shared_data shared{};

    bool headless = false;
    int headless_steps = 2000;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--headless") == 0) {
            headless = true;
        } else if (std::strcmp(argv[i], "--steps") == 0 && i + 1 < argc) {
            headless_steps = std::atoi(argv[++i]);
        }
    }

    const auto find_mjcf_path = []() -> std::filesystem::path {
        const std::vector<std::filesystem::path> relative_paths = {
            "src/mj_kdl_wrapper/third_party/menagerie/kinova_gen3/gen3.xml",
            "mj_kdl_wrapper/third_party/menagerie/kinova_gen3/gen3.xml",
            "third_party/menagerie/kinova_gen3/gen3.xml",
        };
        std::vector<std::filesystem::path> roots = {std::filesystem::current_path()};
        roots.push_back(std::filesystem::path(__FILE__).parent_path());
        for (const auto &start : roots) {
            for (auto root = start; !root.empty(); root = root.parent_path()) {
                for (const auto &relative : relative_paths) {
                    const auto candidate = root / relative;
                    if (std::filesystem::exists(candidate)) {
                        return candidate;
                    }
                }
                if (root == root.root_path()) {
                    break;
                }
            }
        }
        return {};
    };

    const std::filesystem::path mjcf_path = find_mjcf_path();
    if (mjcf_path.empty()) {
        std::cerr << "Kinova Gen3 MJCF not found. Expected mj_kdl_wrapper third_party/menagerie/kinova_gen3/gen3.xml\n";
        return 1;
    }

    mj_kdl::Env mj_env{};
    mj_kdl::SceneSpec mj_scene{};
    mj_kdl::RobotSpec mj_robot_spec{};
    const std::string mjcf_path_string = mjcf_path.string();
    mj_robot_spec.path = mjcf_path_string.c_str();
    mj_scene.robots.push_back(mj_robot_spec);

    if (!mj_kdl::init_env(&mj_env, &mj_scene)) {
        std::cerr << "mj_kdl::init_env() failed for " << mjcf_path_string << "\n";
        return 1;
    }


    mj_kdl::Robot mj_arm_solver{};
    if (!mj_kdl::init_robot_from_mjcf(&mj_arm_solver, mj_env.model, mj_env.data, "base_link", "bracelet_link")) {
        std::cerr << "init_robot_from_mjcf() failed for arm_solver\n";
        mj_kdl::cleanup(&mj_env);
        return 1;
    }
    mj_arm_solver.ctrl_mode = mj_kdl::CtrlMode::TORQUE;
    mj_kdl::env_add_robot(&mj_env, &mj_arm_solver);

    KDL::Chain *chain_arm_solver = &mj_arm_solver.chain;

    robot.arm_solver = manipulator_robot{
        .robot = &mj_arm_solver,
        .chain = chain_arm_solver,
    };


    mj_kdl::Viewer mj_viewer{};
    if (!headless) {
        if (!mj_kdl::init_window_sim(&mj_viewer, &mj_arm_solver)) {
            std::cerr << "mj_kdl::init_window_sim() failed\n";
            mj_kdl::cleanup(&mj_env);
            return 1;
        }
    }
    motion_fall_down_state motion_fall_down_state_instance;


    mj_env.on_reset = [&](mj_kdl::ResetContext *) {
        const double _q_home[7] = {0.0, 0.2618, 3.1416, -2.2689, 0.0, 0.9599, 1.5708};
        KDL::JntArray _q(mj_arm_solver.n_joints);
        for (int _i = 0; _i < mj_arm_solver.n_joints && _i < 7; ++_i)
            _q(_i) = _q_home[_i];
        mj_kdl::set_joint_pos(&mj_arm_solver, _q, false);
        reset_motion_fall_down(motion_fall_down_state_instance);
    };
    mj_kdl::reset(&mj_env);
    double _prev_sim_time = mj_arm_solver.data->time;

    int _step = 0;
    int current_motion = 0;
    while ((!headless || headless_steps-- > 0)) {
        if (mj_arm_solver.data->time < _prev_sim_time - 1e-6) {
            mj_kdl::reset(&mj_env);
            current_motion = 0;
        }
        _prev_sim_time = mj_arm_solver.data->time;
        switch (current_motion) {
        case 0: {
            update_motion_fall_down(motion_fall_down_state_instance, shared, robot);
            monitor_motion_fall_down(motion_fall_down_state_instance, shared);
            if (!motion_fall_down_state_instance.active && can_start_motion_fall_down(motion_fall_down_state_instance, shared)) {
                motion_fall_down_state_instance.active = true;
            }
            if (motion_fall_down_state_instance.active) {
                control_motion_fall_down(motion_fall_down_state_instance, shared, robot);
                apply_motion_fall_down(motion_fall_down_state_instance, shared, robot);
                motion_fall_down_state_instance.active = false;
                current_motion = 1;
            }
            break;
        }

        default:
            current_motion = 1 - 1;
            break;
        }
        ++_step;
    }
    if (headless) {
        std::cout << "steps=" << _step << " sim_time=" << mj_arm_solver.data->time << "\n";
        std::cout << "pose_ee p=[" << shared.pose_ee.p.x() << " " << shared.pose_ee.p.y() << " " << shared.pose_ee.p.z() << "]\n";
        std::cout << "twist_ee vel=[" << shared.twist_ee.vel.x() << " " << shared.twist_ee.vel.y() << " " << shared.twist_ee.vel.z() << "] rot=[" << shared.twist_ee.rot.x() << " " << shared.twist_ee.rot.y() << " " << shared.twist_ee.rot.z() << "]\n";
        std::cout << "pose_diff_ctrl_orientation vel=[" << shared.pose_diff_ctrl_orientation.vel.x() << " " << shared.pose_diff_ctrl_orientation.vel.y() << " " << shared.pose_diff_ctrl_orientation.vel.z() << "] rot=[" << shared.pose_diff_ctrl_orientation.rot.x() << " " << shared.pose_diff_ctrl_orientation.rot.y() << " " << shared.pose_diff_ctrl_orientation.rot.z() << "]\n";
        std::cout << "eacc_ctrl_orientation_ang_x=" << shared.eacc_ctrl_orientation_ang_x << "\n";
        std::cout << "eacc_twist_ee_linear_x_fall_down=" << shared.eacc_twist_ee_linear_x_fall_down << "\n";
        std::cout << "vel_y_zero=" << shared.vel_y_zero << "\n";
        std::cout << "eacc_ctrl_orientation_ang_y=" << shared.eacc_ctrl_orientation_ang_y << "\n";
        std::cout << "vel_z_down=" << shared.vel_z_down << "\n";
        std::cout << "eacc_twist_ee_linear_z_fall_down=" << shared.eacc_twist_ee_linear_z_fall_down << "\n";
        std::cout << "eacc_ctrl_orientation_ang_z=" << shared.eacc_ctrl_orientation_ang_z << "\n";
        std::cout << "gravity_vec=" << shared.gravity_vec << "\n";
        std::cout << "pose_ee_orientation=" << shared.pose_ee_orientation << "\n";
        std::cout << "twist_ee_linear_y_err_fall_down=" << shared.twist_ee_linear_y_err_fall_down << "\n";
        std::cout << "twist_ee_linear_x_err_fall_down=" << shared.twist_ee_linear_x_err_fall_down << "\n";
        std::cout << "vel_x_zero=" << shared.vel_x_zero << "\n";
        std::cout << "twist_ee_linear_z_err_fall_down=" << shared.twist_ee_linear_z_err_fall_down << "\n";
        std::cout << "eacc_twist_ee_linear_y_fall_down=" << shared.eacc_twist_ee_linear_y_fall_down << "\n";
        std::cout << "pose_ee_start p=[" << shared.pose_ee_start.p.x() << " " << shared.pose_ee_start.p.y() << " " << shared.pose_ee_start.p.z() << "]\n";
        std::cout << "tau=[";
        for (int _i = 0; _i < robot.arm_solver.robot->n_joints; ++_i)
            std::cout << robot.arm_solver.robot->jnt_trq_cmd[_i] << " ";
        std::cout << "]\n";
    }

    if (!headless) {
        mj_kdl::cleanup(&mj_viewer);
    }
    mj_kdl::cleanup(&mj_env);
    mj_kdl::cleanup(&mj_arm_solver);

    return 0;
}
