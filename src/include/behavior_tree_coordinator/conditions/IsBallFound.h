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

#include <geometry_msgs/msg/pose.hpp>

#include <behaviortree_ros/components/RosCondition.h>

class IsBallFound : public RosCondition
{
public:
    static BT::PortsList providedPorts() { return {BT::InputPort<bool>("next_goal_trigger"),
                                                   BT::OutputPort<float>("ball_x"),
                                                   BT::OutputPort<float>("ball_y"),
                                                   BT::OutputPort<float>("ball_quaternion_x"),
                                                   BT::OutputPort<float>("ball_quaternion_y"),
                                                   BT::OutputPort<float>("ball_quaternion_z"),
                                                   BT::OutputPort<float>("ball_quaternion_w")}; }

    IsBallFound(const std::string &name, const BT::NodeConfiguration &config);

    BT::NodeStatus on_check() override;

private:
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr ball_pose_subscription_;

    geometry_msgs::msg::Pose::SharedPtr ball_pose_ = std::make_shared<geometry_msgs::msg::Pose>();
};
