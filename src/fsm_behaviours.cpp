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
#include <rclcpp/rclcpp.hpp>
#include "coord2b/functions/event_loop.h"
#include "bdd_collab_bhv_cpp/collab_pickplace_fsm.hpp"
#include "bdd_collab_bhv_cpp/fsm_behaviours.hpp"

namespace bcb = bdd_collab_bhv;

bcb::MockupCollabBehaviour::MockupCollabBehaviour(rclcpp::Time pNow, uint pHeartbeatDurMiliSec)
  : mHeartbeatPeriod(std::chrono::milliseconds(pHeartbeatDurMiliSec))
{ mNextHeartbeat = pNow + mHeartbeatPeriod; }

void bcb::MockupCollabBehaviour::step(
  std::shared_ptr<rclcpp::Node> pNodePtr,
  const struct fsm_nbx         *pFsmPtr
)
{
    auto now = pNodePtr->get_clock()->now();
    if (now < mNextHeartbeat) { return; }
    mNextHeartbeat += mHeartbeatPeriod;
    RCLCPP_INFO(
      pNodePtr->get_logger(), "State: %s", pFsmPtr->states[pFsmPtr->currentStateIndex].name
    );

    if (pFsmPtr->currentStateIndex == collab_pickplace::S_TOUCH_TABLE) {
        produce_event(pFsmPtr->eventData, collab_pickplace::E_TABLE_TOUCHED);
    } else if (pFsmPtr->currentStateIndex == collab_pickplace::S_SLIDE) {
        produce_event(pFsmPtr->eventData, collab_pickplace::E_OBJ_REACHED);
    } else if (pFsmPtr->currentStateIndex == collab_pickplace::S_GRASP) {
        produce_event(pFsmPtr->eventData, collab_pickplace::E_GRASP_DONE);
    } else if (pFsmPtr->currentStateIndex == collab_pickplace::S_COLLAB_MOVE) {
        produce_event(pFsmPtr->eventData, collab_pickplace::E_PLACE_REACHED);
    } else if (pFsmPtr->currentStateIndex == collab_pickplace::S_RELEASE) {
        produce_event(pFsmPtr->eventData, collab_pickplace::E_RELEASE_DONE);
    } else if (pFsmPtr->currentStateIndex == collab_pickplace::S_RECOVER) {
        produce_event(pFsmPtr->eventData, collab_pickplace::E_RECOVER_DONE);
    }
}
