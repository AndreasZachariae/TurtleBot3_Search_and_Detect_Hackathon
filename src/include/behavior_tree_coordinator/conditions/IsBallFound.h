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
#include <behaviortree_ros/tools/convert.h>

#include <std_msgs/msg/bool.hpp>

#include <behaviortree_ros/components/RosCondition.h>

class IsBallFound : public RosCondition
{
public:
    static BT::PortsList providedPorts() { return {BT::InputPort<bool>("next_goal_trigger")}; }

    IsBallFound(const std::string &name, const BT::NodeConfiguration &config);

    BT::NodeStatus on_check() override;

private:
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ball_found_subscription_;
    bool is_ball_found = false;
};
