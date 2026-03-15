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

#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/server.hpp>
#include "bdd_demo_behaviours_cpp/human_pickplace_mockup_node.hpp"
#include "bdd_demo_behaviours_cpp/conversions.hpp"
#include "bdd_ros2_interfaces/action/behaviour.hpp"

namespace bdb = bdd_demo_bhv;

bdb::HumanPickplaceMockupNode::HumanPickplaceMockupNode(const rclcpp::NodeOptions &pOptions)
  : rclcpp::Node("human_pickplace_mockup_node", pOptions)
{
    declare_parameter<std::string>("bhv_server_name", "bhv_server");
    const std::string serverName = get_parameter("bhv_server_name").as_string();

    auto goalHandler = [this](const rclcpp_action::GoalUUID &pUUID,
                         std::shared_ptr<const Behaviour::Goal> pGoalPtr) {
        RCLCPP_INFO(this->get_logger(),
          "Received request for scenario: %s",
          uuid_to_hex(pGoalPtr->scenario_context_id).c_str());
        RCLCPP_INFO(this->get_logger(),
          "Received Behaviour goal with %zu parameters",
          pGoalPtr->parameters.size());
        for (const auto &param_val : pGoalPtr->parameters) {
            RCLCPP_INFO(
              this->get_logger(), "- Parameter relation: %s", param_val.param_rel_uri.c_str());
            for (const auto &val_uri : param_val.param_val_uris) {
                RCLCPP_INFO(this->get_logger(), "  + Parameter value: %s", val_uri.c_str());
            }
        }
        (void)pUUID;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    };

    auto cancelHandler = [this](const std::shared_ptr<GoalHandleBehaviour> pGoalHandlePtr) {
        RCLCPP_INFO(this->get_logger(),
          "Received cancel request for scenario: %s",
          uuid_to_hex(pGoalHandlePtr->get_goal()->scenario_context_id).c_str());
        return rclcpp_action::CancelResponse::ACCEPT;
    };

    auto accepted_handler = [this](const std::shared_ptr<GoalHandleBehaviour> pGoalHandlePtr) {
        std::thread([this, pGoalHandlePtr]() { this->execute(pGoalHandlePtr); }).detach();
    };

    mBhvServerPtr = rclcpp_action::create_server<Behaviour>(
      this, serverName, goalHandler, cancelHandler, accepted_handler);
}

void bdb::HumanPickplaceMockupNode::execute(
  const std::shared_ptr<GoalHandleBehaviour> &pGoalHandlePtr)
{}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<bdb::HumanPickplaceMockupNode>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}
