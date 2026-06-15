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
#include <rclcpp_action/rclcpp_action.hpp>
#include "rclcpp/publisher.hpp"
#include "unique_identifier_msgs/msg/uuid.hpp"
#include "coord2b/types/fsm.h"
#include "bdd_ros2_interfaces/msg/trinary_stamped.hpp"
#include "bdd_ros2_interfaces/action/behaviour.hpp"

namespace bdd_collab_bhv {

class BehaviourInterface
{
  public:
    using UUID                = unique_identifier_msgs::msg::UUID;
    using Behaviour           = bdd_ros2_interfaces::action::Behaviour;
    using GoalHandleBehaviour = rclcpp_action::ServerGoalHandle<Behaviour>;

    virtual void step(
      std::shared_ptr<rclcpp::Node>        pNodePtr,
      const struct fsm_nbx                *pFsmPtr,
      const UUID                          &pScenarioContextId = UUID(),
      std::shared_ptr<GoalHandleBehaviour> pGoalHandlePtr     = nullptr
    )                             = 0;
    virtual ~BehaviourInterface() = default;
};

class MockupCollabBehaviour : public BehaviourInterface
{
  public:
    using TrinaryStamped = bdd_ros2_interfaces::msg::TrinaryStamped;

    MockupCollabBehaviour(
      rclcpp::Time                                 pNow,
      uint                                         pHeartbeatDurMiliSec,
      rclcpp::Publisher<TrinaryStamped>::SharedPtr pLocatedPickPublisher,
      rclcpp::Publisher<TrinaryStamped>::SharedPtr pIsHeldPublisher,
      rclcpp::Publisher<TrinaryStamped>::SharedPtr pLocatedPlacePublisher
    );

    void step(
      std::shared_ptr<rclcpp::Node>        pNodePtr,
      const struct fsm_nbx                *pFsmPtr,
      const UUID                          &pScenarioContextId = UUID(),
      std::shared_ptr<GoalHandleBehaviour> pGoalHandlePtr     = nullptr
    ) override;

  private:
    rclcpp::Duration mHeartbeatPeriod;
    rclcpp::Time     mNextHeartbeat;

    std::shared_ptr<Behaviour::Feedback> mFeedbackPtr;
    std::shared_ptr<Behaviour::Result>   mResponsePtr;

    rclcpp::Publisher<TrinaryStamped>::SharedPtr mLocatedPickPublisher;
    rclcpp::Publisher<TrinaryStamped>::SharedPtr mIsHeldPublisher;
    rclcpp::Publisher<TrinaryStamped>::SharedPtr mLocatedPlacePublisher;

    TrinaryStamped mLocatedPickMsg;
    TrinaryStamped mIsHeldMsg;
    TrinaryStamped mLocatedPlaceMsg;
};

} // namespace bdd_collab_bhv

#endif // BDD_COLLAB_BHV_CPP__FSM_BEHAVIOURS_HPP_
