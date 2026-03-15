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

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "bdd_ros2_interfaces/action/behaviour.hpp"

namespace bdd_demo_bhv {
class HumanPickplaceMockupNode : public rclcpp::Node
{
  public:
    using Behaviour = bdd_ros2_interfaces::action::Behaviour;
    using GoalHandleBehaviour = rclcpp_action::ServerGoalHandle<Behaviour>;

    explicit HumanPickplaceMockupNode(const rclcpp::NodeOptions &pOptions = rclcpp::NodeOptions());

    void execute(const std::shared_ptr<GoalHandleBehaviour> &pGoalHandlePtr);

  private:
    rclcpp_action::Server<Behaviour>::SharedPtr mBhvServerPtr;
};// HumanPickplaceMockupNode
}// namespace bdd_demo_bhv
#endif// BDD_DEMO_BEHAVIOURS_CPP__HUMAN_PICKPLACE_MOCKUP_NODE_HPP_
