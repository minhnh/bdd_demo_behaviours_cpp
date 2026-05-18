#include "runtime.hpp"
#include "shared_state.hpp"
#include "motion_home.hpp"
#include "motion_pick_above.hpp"
#include "motion_pick.hpp"
#include "motion_grasp_hold__close_grip.hpp"
#include "motion_lift.hpp"
#include "motion_place_above.hpp"
#include "motion_place.hpp"
#include "motion_grasp_hold__open_grip.hpp"
#include "motion_retreat.hpp"


#include <cstring>
#include <deque>
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

    const auto find_asset_path = [](const std::vector<std::filesystem::path> &relative_paths) -> std::filesystem::path {
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

    const std::filesystem::path mjcf_path = find_asset_path({"src/mj_kdl_wrapper/third_party/menagerie/kinova_gen3/gen3.xml"});
    if (mjcf_path.empty()) {
        std::cerr << "MJCF model not found: src/mj_kdl_wrapper/third_party/menagerie/kinova_gen3/gen3.xml\n";
        return 1;
    }
    mj_kdl::Env mj_env{};
    mj_kdl::SceneSpec mj_scene{};
    mj_kdl::RobotSpec mj_robot_spec{};
    std::deque<std::string> mj_path_storage;
    const std::string mjcf_path_string = mjcf_path.string();
    mj_robot_spec.path = mjcf_path_string.c_str();
    mj_robot_spec.pos[0] = 0.0;
    mj_robot_spec.pos[1] = 0.0;
    mj_robot_spec.pos[2] = 0.72;
    {
        const std::filesystem::path attachment_path = find_asset_path({"src/mj_kdl_wrapper/third_party/menagerie/robotiq_2f85/2f85.xml"});
        if (attachment_path.empty()) {
            std::cerr << "MJCF attachment not found: src/mj_kdl_wrapper/third_party/menagerie/robotiq_2f85/2f85.xml\n";
            return 1;
        }
        mj_path_storage.push_back(attachment_path.string());
        mj_kdl::AttachmentSpec attachment_spec{};
        attachment_spec.mjcf_path = mj_path_storage.back().c_str();
        attachment_spec.attach_to = "bracelet_link";
        attachment_spec.prefix = "g_";
        attachment_spec.pos[0] = 0.0;
        attachment_spec.pos[1] = 0.0;
        attachment_spec.pos[2] = -0.061525;
        attachment_spec.euler[0] = 180.0;
        attachment_spec.euler[1] = 0.0;
        attachment_spec.euler[2] = 0.0;
        mj_robot_spec.attachments.push_back(attachment_spec);
    }

    mj_scene.robots.push_back(mj_robot_spec);
    {
        mj_kdl::SceneObject object_spec{};
        object_spec.name = "table";
        const std::filesystem::path object_path = find_asset_path({"src/mj_kdl_wrapper/src/examples/assets/table.xml"});
        if (object_path.empty()) {
            std::cerr << "MJCF object not found: src/mj_kdl_wrapper/src/examples/assets/table.xml\n";
            return 1;
        }
        object_spec.mjcf_path = object_path.string();
        object_spec.pos[0] = 0.0;
        object_spec.pos[1] = 0.0;
        object_spec.pos[2] = 0.72;
        object_spec.fixed = true;
        object_spec.shape = mj_kdl::Shape::BOX;
        object_spec.size[0] = 0.03;
        object_spec.size[1] = 0.03;
        object_spec.size[2] = 0.03;
        object_spec.mass = 0.1;
        object_spec.friction[0] = 0.5;
        object_spec.friction[1] = 0.005;
        object_spec.friction[2] = 1.0E-4;
        mj_scene.objects.push_back(object_spec);
    }

    {
        mj_kdl::SceneObject object_spec{};
        object_spec.name = "cube";
        object_spec.pos[0] = 0.5;
        object_spec.pos[1] = 0.0;
        object_spec.pos[2] = 0.745;
        object_spec.fixed = false;
        object_spec.shape = mj_kdl::Shape::BOX;
        object_spec.size[0] = 0.03;
        object_spec.size[1] = 0.03;
        object_spec.size[2] = 0.03;
        object_spec.mass = 0.1;
        object_spec.friction[0] = 0.8;
        object_spec.friction[1] = 0.01;
        object_spec.friction[2] = 1.0E-4;
        mj_scene.objects.push_back(object_spec);
    }


    if (!mj_kdl::init_env(&mj_env, &mj_scene)) {
        std::cerr << "mj_kdl::init_env() failed for " << mjcf_path_string << "\n";
        return 1;
    }


    mj_kdl::Robot mj_arm_solver{};
    mj_kdl::ToolFrameSpec mj_tool_arm_solver{};
    mj_tool_arm_solver.tool_body = "g_base";
    mj_tool_arm_solver.tcp_site = "g_pinch";
    if (!mj_kdl::init_robot_from_mjcf(&mj_arm_solver, mj_env.model, mj_env.data, "base_link", "bracelet_link", "", &mj_tool_arm_solver)) {
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
    motion_home_state motion_home_state_instance;
    motion_pick_above_state motion_pick_above_state_instance;
    motion_pick_state motion_pick_state_instance;
    motion_grasp_hold__close_grip_state motion_grasp_hold__close_grip_state_instance;
    motion_lift_state motion_lift_state_instance;
    motion_place_above_state motion_place_above_state_instance;
    motion_place_state motion_place_state_instance;
    motion_grasp_hold__open_grip_state motion_grasp_hold__open_grip_state_instance;
    motion_retreat_state motion_retreat_state_instance;


    mj_env.on_reset = [&](mj_kdl::ResetContext *) {
        const double _q_home[7] = {0.0, 0.2618, 3.1416, -2.2689, 0.0, 0.9599, 1.5708};
        KDL::JntArray _q(mj_arm_solver.n_joints);
        for (int _i = 0; _i < mj_arm_solver.n_joints && _i < 7; ++_i)
            _q(_i) = _q_home[_i];
        mj_kdl::set_joint_pos(&mj_arm_solver, _q, false);
        reset_motion_home(motion_home_state_instance);
                reset_motion_pick_above(motion_pick_above_state_instance);
                reset_motion_pick(motion_pick_state_instance);
                reset_motion_grasp_hold__close_grip(motion_grasp_hold__close_grip_state_instance);
                reset_motion_lift(motion_lift_state_instance);
                reset_motion_place_above(motion_place_above_state_instance);
                reset_motion_place(motion_place_state_instance);
                reset_motion_grasp_hold__open_grip(motion_grasp_hold__open_grip_state_instance);
                reset_motion_retreat(motion_retreat_state_instance);
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
            update_motion_home(motion_home_state_instance, shared, robot);
            monitor_motion_home(motion_home_state_instance, shared);
            if (!motion_home_state_instance.active && can_start_motion_home(motion_home_state_instance, shared)) {
                motion_home_state_instance.active = true;
            }
            if (motion_home_state_instance.active) {
                control_motion_home(motion_home_state_instance, shared, robot);
                apply_motion_home(motion_home_state_instance, shared, robot);
                if (motion_spec::runtime::constraint_satisfied(shared.twist_ee_base_linear_z_err)) {
                    motion_home_state_instance.active = false;
                    current_motion = 1;
                }
            }
            break;
        }

        case 1: {
            update_motion_pick_above(motion_pick_above_state_instance, shared, robot);
            monitor_motion_pick_above(motion_pick_above_state_instance, shared);
            if (!motion_pick_above_state_instance.active && can_start_motion_pick_above(motion_pick_above_state_instance, shared)) {
                motion_pick_above_state_instance.active = true;
            }
            if (motion_pick_above_state_instance.active) {
                control_motion_pick_above(motion_pick_above_state_instance, shared, robot);
                apply_motion_pick_above(motion_pick_above_state_instance, shared, robot);
                if (motion_spec::runtime::constraint_satisfied(shared.twist_ee_base_linear_z_err)) {
                    motion_pick_above_state_instance.active = false;
                    current_motion = 2;
                }
            }
            break;
        }

        case 2: {
            update_motion_pick(motion_pick_state_instance, shared, robot);
            monitor_motion_pick(motion_pick_state_instance, shared);
            if (!motion_pick_state_instance.active && can_start_motion_pick(motion_pick_state_instance, shared)) {
                motion_pick_state_instance.active = true;
            }
            if (motion_pick_state_instance.active) {
                control_motion_pick(motion_pick_state_instance, shared, robot);
                apply_motion_pick(motion_pick_state_instance, shared, robot);
                if (motion_spec::runtime::constraint_satisfied(shared.pose_cube_g_pinch_distance_err)) {
                    motion_pick_state_instance.active = false;
                    current_motion = 3;
                }
            }
            break;
        }

        case 3: {
            update_motion_grasp_hold__close_grip(motion_grasp_hold__close_grip_state_instance, shared, robot);
            monitor_motion_grasp_hold__close_grip(motion_grasp_hold__close_grip_state_instance, shared);
            if (!motion_grasp_hold__close_grip_state_instance.active && can_start_motion_grasp_hold__close_grip(motion_grasp_hold__close_grip_state_instance, shared)) {
                motion_grasp_hold__close_grip_state_instance.active = true;
            }
            if (motion_grasp_hold__close_grip_state_instance.active) {
                control_motion_grasp_hold__close_grip(motion_grasp_hold__close_grip_state_instance, shared, robot);
                apply_motion_grasp_hold__close_grip(motion_grasp_hold__close_grip_state_instance, shared, robot);
                motion_grasp_hold__close_grip_state_instance.active = false;
                current_motion = 4;
            }
            break;
        }

        case 4: {
            update_motion_lift(motion_lift_state_instance, shared, robot);
            monitor_motion_lift(motion_lift_state_instance, shared);
            if (!motion_lift_state_instance.active && can_start_motion_lift(motion_lift_state_instance, shared)) {
                motion_lift_state_instance.active = true;
            }
            if (motion_lift_state_instance.active) {
                control_motion_lift(motion_lift_state_instance, shared, robot);
                apply_motion_lift(motion_lift_state_instance, shared, robot);
                if (motion_spec::runtime::constraint_satisfied(shared.twist_ee_base_linear_z_err)) {
                    motion_lift_state_instance.active = false;
                    current_motion = 5;
                }
            }
            break;
        }

        case 5: {
            update_motion_place_above(motion_place_above_state_instance, shared, robot);
            monitor_motion_place_above(motion_place_above_state_instance, shared);
            if (!motion_place_above_state_instance.active && can_start_motion_place_above(motion_place_above_state_instance, shared)) {
                motion_place_above_state_instance.active = true;
            }
            if (motion_place_above_state_instance.active) {
                control_motion_place_above(motion_place_above_state_instance, shared, robot);
                apply_motion_place_above(motion_place_above_state_instance, shared, robot);
                if (motion_spec::runtime::constraint_satisfied(shared.twist_ee_base_linear_z_err)) {
                    motion_place_above_state_instance.active = false;
                    current_motion = 6;
                }
            }
            break;
        }

        case 6: {
            update_motion_place(motion_place_state_instance, shared, robot);
            monitor_motion_place(motion_place_state_instance, shared);
            if (!motion_place_state_instance.active && can_start_motion_place(motion_place_state_instance, shared)) {
                motion_place_state_instance.active = true;
            }
            if (motion_place_state_instance.active) {
                control_motion_place(motion_place_state_instance, shared, robot);
                apply_motion_place(motion_place_state_instance, shared, robot);
                if (motion_spec::runtime::constraint_satisfied(shared.twist_ee_base_linear_z_err)) {
                    motion_place_state_instance.active = false;
                    current_motion = 7;
                }
            }
            break;
        }

        case 7: {
            update_motion_grasp_hold__open_grip(motion_grasp_hold__open_grip_state_instance, shared, robot);
            monitor_motion_grasp_hold__open_grip(motion_grasp_hold__open_grip_state_instance, shared);
            if (!motion_grasp_hold__open_grip_state_instance.active && can_start_motion_grasp_hold__open_grip(motion_grasp_hold__open_grip_state_instance, shared)) {
                motion_grasp_hold__open_grip_state_instance.active = true;
            }
            if (motion_grasp_hold__open_grip_state_instance.active) {
                control_motion_grasp_hold__open_grip(motion_grasp_hold__open_grip_state_instance, shared, robot);
                apply_motion_grasp_hold__open_grip(motion_grasp_hold__open_grip_state_instance, shared, robot);
                motion_grasp_hold__open_grip_state_instance.active = false;
                current_motion = 8;
            }
            break;
        }

        case 8: {
            update_motion_retreat(motion_retreat_state_instance, shared, robot);
            monitor_motion_retreat(motion_retreat_state_instance, shared);
            if (!motion_retreat_state_instance.active && can_start_motion_retreat(motion_retreat_state_instance, shared)) {
                motion_retreat_state_instance.active = true;
            }
            if (motion_retreat_state_instance.active) {
                control_motion_retreat(motion_retreat_state_instance, shared, robot);
                apply_motion_retreat(motion_retreat_state_instance, shared, robot);
                if (motion_spec::runtime::constraint_satisfied(shared.twist_ee_base_linear_z_err)) {
                    motion_retreat_state_instance.active = false;
                    current_motion = 9;
                }
            }
            break;
        }

        default:
            current_motion = 9 - 1;
            break;
        }
        ++_step;
    }
    if (headless) {
        std::cout << "steps=" << _step << " sim_time=" << mj_arm_solver.data->time << "\n";
    }

    if (!headless) {
        mj_kdl::cleanup(&mj_viewer);
    }
    mj_kdl::cleanup(&mj_env);
    mj_kdl::cleanup(&mj_arm_solver);

    return 0;
}
