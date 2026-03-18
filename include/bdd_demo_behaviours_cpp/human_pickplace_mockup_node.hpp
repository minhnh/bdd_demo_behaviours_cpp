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
#include <thread>
#include "coord2b/types/fsm.h"
#include "bdd_ros2_interfaces/action/behaviour.hpp"
#include "bdd_demo_behaviours_cpp/collab_pickplace.fsm.hpp"

#define LOOP_PERIOD_MICRO_SECS 1000// for 1kHz

namespace bdd_demo_bhv {
class HumanPickplaceMockupNode : public rclcpp::Node
{
  public:
    using Behaviour = bdd_ros2_interfaces::action::Behaviour;
    using GoalHandleBehaviour = rclcpp_action::ServerGoalHandle<Behaviour>;

    explicit HumanPickplaceMockupNode(const rclcpp::NodeOptions &pOptions = rclcpp::NodeOptions());

    ~HumanPickplaceMockupNode() override;

  private:
    rclcpp_action::Server<Behaviour>::SharedPtr mBhvServerPtr;

    std::thread mFsmThread;
    std::mutex mGoalMutex;

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
    // use unique_ptr to automatically handle mem cleanup in destructor
    // should only be written to in fsm_loop()
    std::unique_ptr<fsm_nbx, decltype(&destroy_fsm)> mFsmPtr;

    // Functions
    void fsm_loop();

};// HumanPickplaceMockupNode
}// namespace bdd_demo_bhv
#endif// BDD_DEMO_BEHAVIOURS_CPP__HUMAN_PICKPLACE_MOCKUP_NODE_HPP_
