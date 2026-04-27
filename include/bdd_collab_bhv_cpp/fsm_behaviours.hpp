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

#ifndef BDD_COLLAB_BHV_CPP__FSM_BEHAVIOURS_HPP_
#define BDD_COLLAB_BHV_CPP__FSM_BEHAVIOURS_HPP_

#include <memory>
#include <rclcpp/time.hpp>
#include <rclcpp/node.hpp>
#include "coord2b/types/fsm.h"

namespace bdd_collab_bhv {

class BehaviourInterface
{
  public:
    virtual void step(std::shared_ptr<rclcpp::Node> pNodePtr, const struct fsm_nbx *pFsmPtr) = 0;
    virtual ~BehaviourInterface() = default;
};

class MockupCollabBehaviour : public BehaviourInterface
{
  public:
    MockupCollabBehaviour(rclcpp::Time pNow, uint pHeartbeatDurMiliSec);

    void step(std::shared_ptr<rclcpp::Node> pNodePtr, const struct fsm_nbx *pFsmPtr) override;

  private:
    rclcpp::Duration mHeartbeatPeriod;
    rclcpp::Time     mNextHeartbeat;
};

} // namespace bdd_collab_bhv

#endif // BDD_COLLAB_BHV_CPP__FSM_BEHAVIOURS_HPP_
