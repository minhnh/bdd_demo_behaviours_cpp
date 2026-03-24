// Copyright 2026 Minh Nguyen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef BDD_DEMO_BEHAVIOURS_CPP__HUMAN_PICKPLACE_MOCKUP_NODE_HPP_
#define BDD_DEMO_BEHAVIOURS_CPP__HUMAN_PICKPLACE_MOCKUP_NODE_HPP_

#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string_view>
#include <thread>
#include "bdd_ros2_interfaces/action/behaviour.hpp"
#include "bdd_demo_behaviours_cpp/collab_pickplace.fsm.hpp"

namespace bdd_demo_bhv {

enum class ExecutionType { Mockup, Simulation, RealRobot };

constexpr std::string_view exec_type_to_str(ExecutionType pExecType)
{
    switch (pExecType) {
    case ExecutionType::Mockup:
        return "mockup";
    case ExecutionType::Simulation:
        return "simulation";
    case ExecutionType::RealRobot:
        return "real robot";
    default:
        return "unknown";
    }
}

class CollabPickplaceNode : public rclcpp::Node
{
  public:
    using Behaviour = bdd_ros2_interfaces::action::Behaviour;
    using GoalHandleBehaviour = rclcpp_action::ServerGoalHandle<Behaviour>;

    explicit CollabPickplaceNode(const rclcpp::NodeOptions &pOptions = rclcpp::NodeOptions());

    ~CollabPickplaceNode() override;

    void start_fsm();

  private:
    rclcpp_action::Server<Behaviour>::SharedPtr mBhvServerPtr;

    std::thread mFsmThread;
    std::mutex mFsmMutex;
    std::mutex mGoalMutex;
    ExecutionType mExecCtx;

    // Shared data between server handlers & FSM loop
    struct GoalData
    {
        std::weak_ptr<GoalHandleBehaviour> mGoalHandlerPtr;
        Behaviour::Goal mGoalCopy;
    };

    std::optional<GoalData> mPendingGoal;
    std::atomic<bool> mFsmLoopRunning{ false };
    std::atomic<bool> mCancelRequested{ false };
    std::atomic<bool> mFsmIdle{ false };

    // Use unique_ptr to automatically handle mem cleanup in destructor.
    // Should only by modified in fsm_loop().
    std::unique_ptr<fsm_nbx, decltype(&destroy_fsm)> mFsmPtr;

    // Functions
    void fsm_loop();

};// CollabPickplaceNode
}// namespace bdd_demo_bhv
#endif// BDD_DEMO_BEHAVIOURS_CPP__HUMAN_PICKPLACE_MOCKUP_NODE_HPP_
