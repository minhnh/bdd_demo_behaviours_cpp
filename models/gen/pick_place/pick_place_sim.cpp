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

#include <cmath>
#include <cstring>
#include <deque>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>

#include <unistd.h>

enum class State {
    Home,
    PickAbove,
    Pick,
    CloseGrip,
    Lift,
    PlaceAbove,
    Place,
    OpenGrip,
    Retreat,
    Done,
};

static const char *state_name(State s) {
    switch (s) {
    case State::Home:       return "Home";
    case State::PickAbove:  return "PickAbove";
    case State::Pick:       return "Pick";
    case State::CloseGrip:  return "CloseGrip";
    case State::Lift:       return "Lift";
    case State::PlaceAbove: return "PlaceAbove";
    case State::Place:      return "Place";
    case State::OpenGrip:   return "OpenGrip";
    case State::Retreat:    return "Retreat";
    case State::Done:       return "Done";
    }
    return "?";
}

int main(int argc, char **argv) {
    robot_io robot{};
    shared_data shared{};

    bool headless = false;
    int headless_steps = 4000;
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
                if (root == root.root_path()) break;
            }
        }
        return {};
    };

    // --- Scene setup ---------------------------------------------------------

    const std::filesystem::path mjcf_path = find_asset_path({"src/mj_kdl_wrapper/third_party/menagerie/kinova_gen3/gen3.xml"});
    if (mjcf_path.empty()) {
        std::cerr << "MJCF model not found\n";
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
            std::cerr << "Gripper MJCF not found\n";
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
            std::cerr << "Table MJCF not found\n";
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
        object_spec.friction[2] = 1.0e-4;
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
        object_spec.friction[2] = 1.0e-4;
        mj_scene.objects.push_back(object_spec);
    }

    if (!mj_kdl::init_env(&mj_env, &mj_scene)) {
        std::cerr << "mj_kdl::init_env() failed\n";
        return 1;
    }

    mj_kdl::Robot mj_arm_solver{};
    mj_kdl::ToolFrameSpec mj_tool_arm_solver{};
    mj_tool_arm_solver.tool_body = "g_base";
    mj_tool_arm_solver.tcp_site = "g_pinch";
    if (!mj_kdl::init_robot_from_mjcf(&mj_arm_solver, mj_env.model, mj_env.data, "base_link", "bracelet_link", "", &mj_tool_arm_solver)) {
        std::cerr << "init_robot_from_mjcf() failed\n";
        mj_kdl::cleanup(&mj_env);
        return 1;
    }
    mj_arm_solver.ctrl_mode = mj_kdl::CtrlMode::TORQUE;
    mj_kdl::env_add_robot(&mj_env, &mj_arm_solver);
    KDL::Chain *chain_arm_solver = &mj_arm_solver.chain;
    robot.arm_solver = manipulator_robot{.robot = &mj_arm_solver, .chain = chain_arm_solver};

    mj_kdl::Viewer mj_viewer{};
    if (!headless) {
        if (!mj_kdl::init_window_sim(&mj_viewer, &mj_arm_solver)) {
            std::cerr << "mj_kdl::init_window_sim() failed\n";
            mj_kdl::cleanup(&mj_env);
            return 1;
        }
    }

    // --- Motion state instances ----------------------------------------------

    motion_home_state                    s_home{};
    motion_pick_above_state              s_pick_above{};
    motion_pick_state                    s_pick{};
    motion_grasp_hold__close_grip_state  s_close_grip{};
    motion_lift_state                    s_lift{};
    motion_place_above_state             s_place_above{};
    motion_place_state                   s_place{};
    motion_grasp_hold__open_grip_state   s_open_grip{};
    motion_retreat_state                 s_retreat{};

    // --- Reset callback ------------------------------------------------------

    mj_env.on_reset = [&](mj_kdl::ResetContext *) {
        const double q_home[7] = {0.0, 0.2618, 3.1416, -2.2689, 0.0, 0.9599, 1.5708};
        KDL::JntArray q(mj_arm_solver.n_joints);
        for (int i = 0; i < mj_arm_solver.n_joints && i < 7; ++i) q(i) = q_home[i];
        mj_kdl::set_joint_pos(&mj_arm_solver, q, false);
        s_home       = {};
        s_pick_above = {};
        s_pick       = {};
        s_close_grip = {};
        s_lift       = {};
        s_place_above = {};
        s_place      = {};
        s_open_grip  = {};
        s_retreat    = {};
    };
    mj_kdl::reset(&mj_env);

    // --- SM loop -------------------------------------------------------------

    State state = State::Home;
    State prev_state = State::Done;
    double prev_sim_time = mj_arm_solver.data->time;
    int state_enter_step = 0;
    const double control_dt = mj_arm_solver.model ? mj_arm_solver.model->opt.timestep : 0.002;
    int step = 0;

    auto phase_elapsed = [&]() {
        return (step - state_enter_step) * control_dt;
    };
    auto enter_state = [&](State next) {
        state = next;
        state_enter_step = step;
    };
    auto tcp_settled = [&]() {
        return shared.twist_ee_base.vel.Norm() <= 0.03 &&
               shared.twist_ee_base.rot.Norm() <= 0.08;
    };
    auto abs_ok = [](double value, double tolerance) {
        return std::fabs(value) <= tolerance;
    };
    auto pose_ok = [&](double ex, double ey, double ez, double erx, double ery, double erz,
                       double pos_tol, double rot_tol) {
        return abs_ok(ex, pos_tol) && abs_ok(ey, pos_tol) && abs_ok(ez, pos_tol) &&
               abs_ok(erx, rot_tol) && abs_ok(ery, rot_tol) && abs_ok(erz, rot_tol);
    };
    auto done_after = [&](double min_time, double timeout, bool target_ok) {
        return (phase_elapsed() >= min_time && target_ok && tcp_settled()) ||
               (timeout > 0.0 && phase_elapsed() >= timeout);
    };
    while (!headless || headless_steps-- > 0) {
        if (mj_arm_solver.data->time < prev_sim_time - 1e-6) {
            mj_kdl::reset(&mj_env);
            state = State::Home;
            prev_state = State::Done;
            state_enter_step = step;
        }
        prev_sim_time = mj_arm_solver.data->time;

        if (state != prev_state) {
            std::cout << "[" << step << "] -> " << state_name(state) << "\n";
            prev_state = state;
        }

        switch (state) {
        case State::Home: {
            update_motion_home(s_home, shared, robot);
            monitor_motion_home(s_home, shared);
            if (!s_home.active)
                s_home.active = true;
            if (s_home.active) {
                control_motion_home(s_home, shared, robot);
                apply_motion_home(s_home, shared, robot);
                const bool target_ok =
                    shared.pose_diff_ctrl_hold_position.vel.Norm() <= 0.03 &&
                    shared.pose_diff_ctrl_hold_orientation.rot.Norm() <= 0.05;
                if (done_after(1.0, 2.5, target_ok)) {
                    s_home.active = false;
                    enter_state(State::PickAbove);
                }
            }
            break;
        }
        case State::PickAbove: {
            update_motion_pick_above(s_pick_above, shared, robot);
            monitor_motion_pick_above(s_pick_above, shared);
            if (!s_pick_above.active)
                s_pick_above.active = true;
            if (s_pick_above.active) {
                control_motion_pick_above(s_pick_above, shared, robot);
                if (headless && step % 500 == 0) {
                    double cr = 0.0, cp = 0.0, cy = 0.0;
                    shared.pose_ee_base.M.GetRPY(cr, cp, cy);
                    std::cout << "  PickAbove orient current_rpy=(" << cr << ", " << cp << ", " << cy
                              << ") pose_err=("
                              << shared.pose_ee_wrt_cube_distance_x_err_pick_above << ", "
                              << shared.pose_ee_wrt_cube_distance_y_err_pick_above << ", "
                              << shared.pose_ee_wrt_cube_distance_z_err_pick_above << "; "
                              << shared.pose_ee_wrt_cube_rotation_x_err_pick_above << ", "
                              << shared.pose_ee_wrt_cube_rotation_y_err_pick_above << ", "
                              << shared.pose_ee_wrt_cube_rotation_z_err_pick_above << ")\n";
                }
                apply_motion_pick_above(s_pick_above, shared, robot);
                const bool target_ok = pose_ok(
                    shared.pose_ee_wrt_cube_distance_x_err_pick_above,
                    shared.pose_ee_wrt_cube_distance_y_err_pick_above,
                    shared.pose_ee_wrt_cube_distance_z_err_pick_above,
                    shared.pose_ee_wrt_cube_rotation_x_err_pick_above,
                    shared.pose_ee_wrt_cube_rotation_y_err_pick_above,
                    shared.pose_ee_wrt_cube_rotation_z_err_pick_above,
                    0.04, 0.03);
                if (phase_elapsed() >= 8.0 && target_ok && tcp_settled()) {
                    s_pick_above.active = false;
                    enter_state(State::Pick);
                }
            }
            break;
        }
        case State::Pick: {
            update_motion_pick(s_pick, shared, robot);
            monitor_motion_pick(s_pick, shared);
            if (!s_pick.active)
                s_pick.active = true;
            if (s_pick.active) {
                control_motion_pick(s_pick, shared, robot);
                apply_motion_pick(s_pick, shared, robot);
                const bool target_ok =
                    motion_spec::runtime::constraint_satisfied(shared.pose_cube_g_pinch_distance_err) &&
                    pose_ok(shared.pose_ee_wrt_cube_distance_x_err_pick,
                            shared.pose_ee_wrt_cube_distance_y_err_pick,
                            shared.pose_ee_wrt_cube_distance_z_err_pick,
                            shared.pose_ee_wrt_cube_rotation_x_err_pick,
                            shared.pose_ee_wrt_cube_rotation_y_err_pick,
                            shared.pose_ee_wrt_cube_rotation_z_err_pick,
                            0.025, 0.03);
                if (phase_elapsed() >= 5.0 && target_ok && tcp_settled()) {
                    s_pick.active = false;
                    enter_state(State::CloseGrip);
                }
            }
            break;
        }
        case State::CloseGrip: {
            update_motion_grasp_hold__close_grip(s_close_grip, shared, robot);
            monitor_motion_grasp_hold__close_grip(s_close_grip, shared);
            if (!s_close_grip.active)
                s_close_grip.active = true;
            if (s_close_grip.active) {
                control_motion_grasp_hold__close_grip(s_close_grip, shared, robot);
                apply_motion_grasp_hold__close_grip(s_close_grip, shared, robot);
                if (phase_elapsed() >= 1.5) {
                    s_close_grip.active = false;
                    enter_state(State::Lift);
                }
            }
            break;
        }
        case State::Lift: {
            update_motion_lift(s_lift, shared, robot);
            monitor_motion_lift(s_lift, shared);
            if (!s_lift.active)
                s_lift.active = true;
            if (s_lift.active) {
                control_motion_lift(s_lift, shared, robot);
                apply_motion_lift(s_lift, shared, robot);
                const bool target_ok = pose_ok(
                    shared.pose_ee_wrt_cube_distance_x_err_lift,
                    shared.pose_ee_wrt_cube_distance_y_err_lift,
                    shared.pose_ee_wrt_cube_distance_z_err_lift,
                    shared.pose_ee_wrt_cube_rotation_x_err_lift,
                    shared.pose_ee_wrt_cube_rotation_y_err_lift,
                    shared.pose_ee_wrt_cube_rotation_z_err_lift,
                    0.04, 0.03);
                if (done_after(2.5, 8.0, target_ok)) {
                    s_lift.active = false;
                    enter_state(State::PlaceAbove);
                }
            }
            break;
        }
        case State::PlaceAbove: {
            update_motion_place_above(s_place_above, shared, robot);
            monitor_motion_place_above(s_place_above, shared);
            if (!s_place_above.active)
                s_place_above.active = true;
            if (s_place_above.active) {
                control_motion_place_above(s_place_above, shared, robot);
                apply_motion_place_above(s_place_above, shared, robot);
                const bool target_ok = pose_ok(
                    shared.pose_ee_base_distance_x_err_place_above,
                    shared.pose_ee_base_distance_y_err_place_above,
                    shared.pose_ee_base_distance_z_err_place_above,
                    shared.pose_ee_wrt_cube_rotation_x_err_place_above,
                    shared.pose_ee_wrt_cube_rotation_y_err_place_above,
                    shared.pose_ee_wrt_cube_rotation_z_err_place_above,
                    0.04, 0.03);
                if (done_after(4.0, 12.0, target_ok)) {
                    s_place_above.active = false;
                    enter_state(State::Place);
                }
            }
            break;
        }
        case State::Place: {
            update_motion_place(s_place, shared, robot);
            monitor_motion_place(s_place, shared);
            if (!s_place.active)
                s_place.active = true;
            if (s_place.active) {
                control_motion_place(s_place, shared, robot);
                apply_motion_place(s_place, shared, robot);
                const bool target_ok = pose_ok(
                    shared.pose_ee_base_distance_x_err_place,
                    shared.pose_ee_base_distance_y_err_place,
                    shared.pose_ee_base_distance_z_err_place,
                    shared.pose_ee_wrt_cube_rotation_x_err_place,
                    shared.pose_ee_wrt_cube_rotation_y_err_place,
                    shared.pose_ee_wrt_cube_rotation_z_err_place,
                    0.025, 0.03);
                if (done_after(3.5, 14.0, target_ok)) {
                    s_place.active = false;
                    enter_state(State::OpenGrip);
                }
            }
            break;
        }
        case State::OpenGrip: {
            update_motion_grasp_hold__open_grip(s_open_grip, shared, robot);
            monitor_motion_grasp_hold__open_grip(s_open_grip, shared);
            if (!s_open_grip.active)
                s_open_grip.active = true;
            if (s_open_grip.active) {
                control_motion_grasp_hold__open_grip(s_open_grip, shared, robot);
                apply_motion_grasp_hold__open_grip(s_open_grip, shared, robot);
                if (phase_elapsed() >= 1.0) {
                    s_open_grip.active = false;
                    enter_state(State::Retreat);
                }
            }
            break;
        }
        case State::Retreat: {
            update_motion_retreat(s_retreat, shared, robot);
            monitor_motion_retreat(s_retreat, shared);
            if (!s_retreat.active)
                s_retreat.active = true;
            if (s_retreat.active) {
                control_motion_retreat(s_retreat, shared, robot);
                apply_motion_retreat(s_retreat, shared, robot);
                const bool target_ok = pose_ok(
                    shared.pose_ee_base_distance_x_err_retreat,
                    shared.pose_ee_base_distance_y_err_retreat,
                    shared.pose_ee_base_distance_z_err_retreat,
                    shared.pose_ee_wrt_cube_rotation_x_err_retreat,
                    shared.pose_ee_wrt_cube_rotation_y_err_retreat,
                    shared.pose_ee_wrt_cube_rotation_z_err_retreat,
                    0.04, 0.08);
                if (done_after(2.0, 6.0, target_ok)) {
                    s_retreat.active = false;
                    enter_state(State::Done);
                }
            }
            break;
        }
        case State::Done:
            if (headless) goto done;
            break;
        }

        ++step;
    }

done:
    if (headless)
        std::cout << "done: steps=" << step << " sim_time=" << mj_arm_solver.data->time << "\n";

    if (!headless) mj_kdl::cleanup(&mj_viewer);
    mj_kdl::cleanup(&mj_env);
    mj_kdl::cleanup(&mj_arm_solver);
    return 0;
}
