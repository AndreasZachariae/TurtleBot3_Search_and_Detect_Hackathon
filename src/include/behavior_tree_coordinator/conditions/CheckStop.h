/** *******************************************************
 * PeTRA - University of Applied Sciences Karlsruhe
 * Module : ROS2-Node "neobotixCoordinator"
 * Purpose : Checks the ROS2-Topic "Stop"
 *
 * @author Andreas Zachariae
 * @author Moritz Weisenböhler
 * @since 1.0.0 (2020.08.26)
 *********************************************************/
#pragma once

#include <behavior_tree_coordinator/default.h>

#include <std_msgs/msg/empty.hpp>

#include <behaviortree_ros/components/RosCondition.h>

class CheckStop : public RosCondition
{
public:
    static BT::PortsList providedPorts() { return BT::PortsList(); }

    CheckStop(const std::string &name, const BT::NodeConfiguration &config);

    BT::NodeStatus on_check() override;

private:
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stop_subscription_;

    bool stop_recieved_ = false;
};
