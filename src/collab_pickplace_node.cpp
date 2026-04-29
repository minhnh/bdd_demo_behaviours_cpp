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
#include <csignal>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <rclcpp/logging.hpp>
#include <stdexcept>
#include <string>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/server.hpp>
#include <rclcpp/utilities.hpp>
#include <utility>
#include "bdd_collab_bhv_cpp/collab_pickplace.fsm.hpp"
#include "coord2b/functions/event_loop.h"
#include "coord2b/functions/fsm.h"
#include "bdd_ros2_interfaces/action/behaviour.hpp"
#include "bdd_collab_bhv_cpp/conversions.hpp"
#include "bdd_collab_bhv_cpp/fsm_behaviours.hpp"
#include "bdd_collab_bhv_cpp/collab_pickplace_node.hpp"

#define TICK_MILI_SECS 1        // 1kHz
#define HEARTBEAT_MILI_SECS 500 // 2Hz

namespace bcb = bdd_collab_bhv;

namespace {

std::string invalid_exec_context_message(const std::string &exec_ctx)
{
    std::ostringstream oss;
    oss << "Invalid 'exec_context' param: '" << exec_ctx << "'";
    return oss.str();
}

std::string unhandled_exec_context_message(bcb::ExecutionType exec_ctx)
{
    std::ostringstream oss;
    oss << "Unhandled execution context type: '" << bcb::exec_type_to_str(exec_ctx) << "'";
    return oss.str();
}

} // namespace

bcb::CollabPickplaceNode::CollabPickplaceNode(const rclcpp::NodeOptions &pOptions)
  : rclcpp::Node("human_pickplace_mockup_node", pOptions), mFsmPtr(create_fsm(), &destroy_fsm)
{
    if (!mFsmPtr) throw std::runtime_error("FSM creation failed");

    declare_parameter<std::string>("bhv_server_name", "bhv_server");
    declare_parameter<std::string>("exec_context", "mockup");

    auto execCtxStr = get_parameter("exec_context").as_string();
    if (execCtxStr.compare("mockup") == 0) {
        mExecCtx = ExecutionType::Mockup;
    } else if (execCtxStr.compare("sim") == 0) {
        mExecCtx = ExecutionType::Simulation;
    } else if (execCtxStr.compare("real") == 0) {
        mExecCtx = ExecutionType::RealRobot;
    } else {
        throw std::runtime_error(invalid_exec_context_message(execCtxStr));
    }
    RCLCPP_INFO(this->get_logger(), "Execution context: %s", execCtxStr.c_str());

    auto goalHandler = [this](
                         const rclcpp_action::GoalUUID         &pUUID,
                         std::shared_ptr<const Behaviour::Goal> pGoalPtr
                       ) {
        (void)pUUID;

        if (mFsmIdle.load(std::memory_order_acquire)) {
            RCLCPP_INFO(
              this->get_logger(),
              "Received request for scenario: %s",
              uuid_to_hex(pGoalPtr->scenario_context_id).c_str()
            );
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        RCLCPP_WARN(
          this->get_logger(),
          "Behaviour not idle, rejecting scenario: %s",
          uuid_to_hex(pGoalPtr->scenario_context_id).c_str()
        );
        return rclcpp_action::GoalResponse::REJECT;
    };

    auto cancelHandler = [this](const std::shared_ptr<GoalHandleBehaviour> pGoalHandlePtr) {
        RCLCPP_INFO(
          this->get_logger(),
          "Received cancel request for scenario: %s",
          uuid_to_hex(pGoalHandlePtr->get_goal()->scenario_context_id).c_str()
        );
        mCancelRequested.store(true, std::memory_order_release);
        return rclcpp_action::CancelResponse::ACCEPT;
    };

    auto accepted_handler = [this](const std::shared_ptr<GoalHandleBehaviour> pGoalHandlePtr) {
        std::lock_guard<std::mutex> lock(mGoalMutex);
        GoalData                    data;
        data.mGoalHandlerPtr = pGoalHandlePtr;
        data.mGoalCopy       = *pGoalHandlePtr->get_goal();
        mPendingGoal         = std::move(data);
    };

    const std::string serverName = get_parameter("bhv_server_name").as_string();
    mBhvServerPtr                = rclcpp_action::create_server<Behaviour>(
      this, serverName, goalHandler, cancelHandler, accepted_handler
    );
}

void bcb::CollabPickplaceNode::start_fsm()
{
    // separate start function from constructor to avoid
    // race condition when calling shared_from_this() in fsm_loop()
    mFsmLoopRunning.store(true);
    mFsmThread = std::thread(&CollabPickplaceNode::fsm_loop, this);
}

bcb::CollabPickplaceNode::~CollabPickplaceNode()
{
    mFsmLoopRunning.store(false);
    // join threads to ensure member destructor calls,
    // specifically the dynamically allocated FSM data.
    if (mFsmThread.joinable()) { mFsmThread.join(); }
}

void bcb::CollabPickplaceNode::fsm_loop()
{
    auto nodePtr = this->shared_from_this();

    std::optional<GoalData> activeGoal;
    bool                    processingGoal = false;

    auto period  = rclcpp::Duration(std::chrono::milliseconds(TICK_MILI_SECS));
    auto now     = this->get_clock()->now();
    auto nxtTick = now + period;

    std::unique_ptr<BehaviourInterface> bhvInfPtr = nullptr;
    switch (mExecCtx) {
    case ExecutionType::Mockup:
        bhvInfPtr = std::make_unique<MockupCollabBehaviour>(now, HEARTBEAT_MILI_SECS);
        break;
    default:
        throw std::runtime_error(unhandled_exec_context_message(mExecCtx));
    }

    while (rclcpp::ok() && mFsmLoopRunning.load(std::memory_order_acquire)) {
        if (!processingGoal) {
            std::lock_guard<std::mutex> lock(mGoalMutex);
            activeGoal = std::move(mPendingGoal);
            mPendingGoal.reset();
        }

        {
            std::lock_guard<std::mutex> lockFsm(mFsmMutex);
            produce_event(mFsmPtr->eventData, E_STEP);

            // Goal handling
            if (activeGoal && !processingGoal) {
                produce_event(mFsmPtr->eventData, E_NEW_GOAL);
                processingGoal = true;
            }
            if (consume_event(mFsmPtr->eventData, E_GOAL_FINISHED)) {
                processingGoal = false;
                activeGoal.reset();
            }

            mFsmIdle.store(mFsmPtr->currentStateIndex == S_IDLE, std::memory_order_release);

            // Behaviour & FSM update
            bhvInfPtr->step(nodePtr, mFsmPtr.get());
            fsm_step_nbx(mFsmPtr.get());
            reconfig_event_buffers(mFsmPtr->eventData);
        }

        // ensure loop rate
        while (now < nxtTick) now = this->get_clock()->now();
        while (nxtTick < now) nxtTick += period;
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto nodePtr = std::make_shared<bcb::CollabPickplaceNode>(rclcpp::NodeOptions());
    nodePtr->start_fsm();
    rclcpp::spin(nodePtr);
    rclcpp::shutdown();
    return 0;
}
