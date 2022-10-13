/** *******************************************************
 * PeTRA - University of Applied Sciences Karlsruhe
 * Module : ROS2-Node "neobotixCoordinator"
 * Purpose : Checks the ROS2-Topic "BatteryState"
 *
 * @author Andreas Zachariae
 * @author Moritz Weisenböhler
 * @since 1.0.0 (2020.08.26)
 *********************************************************/
#pragma once

#include <behavior_tree_coordinator/default.h>

#include <std_msgs/msg/string.hpp>

#include <behaviortree_ros/components/RosCondition.h>

class IsGoalDone : public RosCondition
{
public:
    static BT::PortsList providedPorts() { return {BT::OutputPort<bool>("next_goal_trigger")}; }

    IsGoalDone(const std::string &name, const BT::NodeConfiguration &config);

    BT::NodeStatus on_check() override;

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr goal_finished_subscription_;

    bool new_goal_finished_msg_ = false;
};
