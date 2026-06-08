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
#include <format>
#include <rclcpp/rclcpp.hpp>
#include "coord2b/functions/event_loop.h"
#include "bdd_collab_bhv_cpp/collab_pickplace_fsm.hpp"
#include "bdd_collab_bhv_cpp/fsm_behaviours.hpp"

namespace bcb = bdd_collab_bhv;

bcb::MockupCollabBehaviour::MockupCollabBehaviour(
  rclcpp::Time                                 pNow,
  uint                                         pHeartbeatDurMiliSec,
  rclcpp::Publisher<TrinaryStamped>::SharedPtr pLocatedPickPublisher,
  rclcpp::Publisher<TrinaryStamped>::SharedPtr pIsHeldPublisher,
  rclcpp::Publisher<TrinaryStamped>::SharedPtr pLocatedPlacePublisher
)
  : mHeartbeatPeriod(std::chrono::milliseconds(pHeartbeatDurMiliSec)),
    mFeedbackPtr(std::make_shared<Behaviour::Feedback>()),
    mLocatedPickPublisher(pLocatedPickPublisher), mIsHeldPublisher(pIsHeldPublisher),
    mLocatedPlacePublisher(pLocatedPlacePublisher)
{ mNextHeartbeat = pNow + mHeartbeatPeriod; }

void bcb::MockupCollabBehaviour::step(
  std::shared_ptr<rclcpp::Node>            pNodePtr,
  const struct fsm_nbx                    *pFsmPtr,
  const unique_identifier_msgs::msg::UUID &pScenarioContextId,
  std::shared_ptr<GoalHandleBehaviour>     pGoalHandlePtr
)
{
    auto now = pNodePtr->get_clock()->now();
    if (now < mNextHeartbeat) { return; }
    mNextHeartbeat += mHeartbeatPeriod;
    RCLCPP_INFO(
      pNodePtr->get_logger(), "State: %s", pFsmPtr->states[pFsmPtr->currentStateIndex].name
    );

    if (!pScenarioContextId.uuid.empty() && pGoalHandlePtr) {
        mFeedbackPtr->scenario_context_id = pScenarioContextId;
        mFeedbackPtr->status =
          std::format("current state: {}", pFsmPtr->states[pFsmPtr->currentStateIndex].name);
        pGoalHandlePtr->publish_feedback(mFeedbackPtr);
    }

    TrinaryStamped trinaryMsg;
    trinaryMsg.stamp               = now;
    trinaryMsg.scenario_context_id = pScenarioContextId;
    trinaryMsg.trinary.value       = bdd_ros2_interfaces::msg::Trinary::TRUE;

    if (pFsmPtr->currentStateIndex == collab_pickplace::S_TOUCH_TABLE) {
        mLocatedPickPublisher->publish(trinaryMsg);
        produce_event(pFsmPtr->eventData, collab_pickplace::E_TABLE_TOUCHED);
    } else if (pFsmPtr->currentStateIndex == collab_pickplace::S_SLIDE) {
        produce_event(pFsmPtr->eventData, collab_pickplace::E_OBJ_REACHED);
    } else if (pFsmPtr->currentStateIndex == collab_pickplace::S_GRASP) {
        mIsHeldPublisher->publish(trinaryMsg);
        produce_event(pFsmPtr->eventData, collab_pickplace::E_GRASP_DONE);
    } else if (pFsmPtr->currentStateIndex == collab_pickplace::S_COLLAB_MOVE) {
        produce_event(pFsmPtr->eventData, collab_pickplace::E_PLACE_REACHED);
    } else if (pFsmPtr->currentStateIndex == collab_pickplace::S_RELEASE) {
        mLocatedPlacePublisher->publish(trinaryMsg);
        produce_event(pFsmPtr->eventData, collab_pickplace::E_RELEASE_DONE);
    } else if (pFsmPtr->currentStateIndex == collab_pickplace::S_RECOVER) {
        produce_event(pFsmPtr->eventData, collab_pickplace::E_RECOVER_DONE);
    }
}
