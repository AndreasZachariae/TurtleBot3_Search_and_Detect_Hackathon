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

#include <sensor_msgs/msg/battery_state.hpp>

#include <behaviortree_ros/components/RosCondition.h>

class CheckBattery : public RosCondition
{
public:
    static BT::PortsList providedPorts() { return BT::PortsList(); }

    CheckBattery(const std::string &name, const BT::NodeConfiguration &config);

    BT::NodeStatus on_check() override;

private:
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr battery_state_subscription_;

    float battery_percentage_ = 1;
};
