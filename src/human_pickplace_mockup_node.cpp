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

#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/server.hpp>
#include <rclcpp/utilities.hpp>
#include <utility>
#include "bdd_ros2_interfaces/action/behaviour.hpp"
#include "bdd_demo_behaviours_cpp/human_pickplace_mockup_node.hpp"
#include "bdd_demo_behaviours_cpp/conversions.hpp"
#include "bdd_demo_behaviours_cpp/collab_pickplace.fsm.hpp"

namespace bdb = bdd_demo_bhv;

bdb::HumanPickplaceMockupNode::HumanPickplaceMockupNode(const rclcpp::NodeOptions &pOptions)
  : rclcpp::Node("human_pickplace_mockup_node", pOptions), mFsmPtr(create_fsm(), &destroy_fsm)
{
    if (!mFsmPtr) throw std::runtime_error("FSM creation failed");

    declare_parameter<std::string>("bhv_server_name", "bhv_server");
    const std::string serverName = get_parameter("bhv_server_name").as_string();

    auto goalHandler = [this](const rclcpp_action::GoalUUID &pUUID,
                         std::shared_ptr<const Behaviour::Goal> pGoalPtr) {
        (void)pUUID;

        if (mFsmIdle.load(std::memory_order_acquire)) {
            RCLCPP_INFO(this->get_logger(),
              "Received request for scenario: %s",
              uuid_to_hex(pGoalPtr->scenario_context_id).c_str());
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        RCLCPP_WARN(this->get_logger(),
          "Behaviour not idle, rejecting scenario: %s",
          uuid_to_hex(pGoalPtr->scenario_context_id).c_str());
        return rclcpp_action::GoalResponse::REJECT;
    };

    auto cancelHandler = [this](const std::shared_ptr<GoalHandleBehaviour> pGoalHandlePtr) {
        RCLCPP_INFO(this->get_logger(),
          "Received cancel request for scenario: %s",
          uuid_to_hex(pGoalHandlePtr->get_goal()->scenario_context_id).c_str());
        mCancelRequested.store(true, std::memory_order_release);
        return rclcpp_action::CancelResponse::ACCEPT;
    };

    auto accepted_handler = [this](const std::shared_ptr<GoalHandleBehaviour> pGoalHandlePtr) {
        std::lock_guard<std::mutex> lock(mGoalMutex);
        GoalData data;
        data.mGoalHandlerPtr = pGoalHandlePtr;
        data.mGoalCopy = *pGoalHandlePtr->get_goal();
        mPendingGoal = std::move(data);
    };

    mBhvServerPtr = rclcpp_action::create_server<Behaviour>(
      this, serverName, goalHandler, cancelHandler, accepted_handler);

    mFsmLoopRunning.store(true);
    mFsmThread = std::thread(&HumanPickplaceMockupNode::fsm_loop, this);
}

bdb::HumanPickplaceMockupNode::~HumanPickplaceMockupNode()
{
    mFsmLoopRunning.store(false);
    if (mFsmThread.joinable()) { mFsmThread.join(); }
}

void bdb::HumanPickplaceMockupNode::fsm_loop()
{
    using clock = std::chrono::steady_clock;
    constexpr auto period = std::chrono::microseconds(LOOP_PERIOD_MICRO_SECS);

    std::optional<GoalData> activeGoal;
    auto nxtTick = clock::now();

    while (rclcpp::ok() && mFsmLoopRunning.load(std::memory_order_acquire)) {
        nxtTick += period;

        activeGoal = std::move(mPendingGoal);

        if (activeGoal) {}

        fsm_step_nbx(mFsmPtr.get());
        reconfig_event_buffers(mFsmPtr->eventData);

        std::this_thread::sleep_until(nxtTick);
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<bdb::HumanPickplaceMockupNode>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}
