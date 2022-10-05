/** *******************************************************
 * PeTRA - University of Applied Sciences Karlsruhe
 * Module : ROS2-Node "neobotixCoordinator"
 * Purpose : Provides the ROS2-Action client "ChargeBattery"
 *
 * @author Andreas Zachariae
 * @author Moritz Weisenböhler
 * @since 1.0.0 (2020.08.26)
 *********************************************************/
#pragma once

#include <behavior_tree_coordinator/default.h>

#include <behaviortree_ros/components/RosAction.h>
#include <behaviortree_ros/tools/convert.h>
#include <petra_interfaces/action/charge_battery.hpp>

using ChargeBattery = petra_interfaces::action::ChargeBattery;

class BatteryCharging : public RosAction<ChargeBattery>
{
public:
    static BT::PortsList providedPorts() { return {BT::InputPort<float>("goal_percentage")}; }

    BatteryCharging(const std::string &name, const BT::NodeConfiguration &config) : RosAction(name, config) {}

    std::string ros_name() { return "ChargeBattery"; }

    void on_send(ChargeBattery::Goal &goal) override;
    void on_feedback(const std::shared_ptr<const ChargeBattery::Feedback> feedback) override;
    void on_result(const rclcpp_action::ClientGoalHandle<ChargeBattery>::WrappedResult &result, const ChargeBattery::Goal &goal) override;
};