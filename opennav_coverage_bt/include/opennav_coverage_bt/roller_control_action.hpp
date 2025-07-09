// Copyright (c) 2018 Intel Corporation
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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__ROLLER_CONTROL_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__ROLLER_CONTROL_ACTION_HPP_

#include <string>
#include <vector>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "zbot_interfaces/action/custom_program_action.hpp"

namespace nav2_behavior_tree
{

/**
 * @brief A nav2_behavior_tree::BtActionNode class that wraps zbot_interfaces::action::RollerControl
 */
class RollerControlAction : public BtActionNode<zbot_interfaces::action::CustomProgramAction>
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::RollerControlAction
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  RollerControlAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Function to perform some user-defined operation on tick
   */
  void on_tick() override;
  BT::NodeStatus on_success() override;


  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("pose", "Path created by ComputePathToPose node"),
        BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>(
                  "poses",
                  "Destinations to plan through"),
        BT::InputPort<int>("task_id", 0, "task id"),
        BT::InputPort<std::string>(
        "task_name",
        "task name: eg roller_control"),
        BT::InputPort<std::string>(
        "param_json",
        "parameter in json string"),
        BT::InputPort<std::string>(
          "task_param",
          "Destinations to plan through"),
      });
  }

private:
  bool is_recovery_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__ROLLER_CONTROL_ACTION_HPP_
