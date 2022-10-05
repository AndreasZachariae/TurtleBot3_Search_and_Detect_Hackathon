/** *******************************************************
 * PeTRA - University of Applied Sciences Karlsruhe
 * Module : ROS2-Node "neobotixCoordinator"
 * Purpose : Provides the ROS2-Action client "NavigateToPose"
 *
 * @author Andreas Zachariae
 * @author Moritz Weisenböhler
 * @since 1.0.0 (2020.08.26)
 *********************************************************/
#pragma once

#include <behavior_tree_coordinator/default.h>

#include <behaviortree_ros/components/RosAction.h>
#include <behaviortree_ros/tools/convert.h>
#include <nav2_msgs/action/navigate_to_pose.hpp>

using NavigateToPose = nav2_msgs::action::NavigateToPose;

class BaseMovement : public RosAction<NavigateToPose>
{
public:
    static BT::PortsList providedPorts() { return {BT::InputPort<float>("x"),
                                                   BT::InputPort<float>("y"),
                                                   BT::InputPort<float>("quaternion_x"),
                                                   BT::InputPort<float>("quaternion_y"),
                                                   BT::InputPort<float>("quaternion_z"),
                                                   BT::InputPort<float>("quaternion_w")}; }

    BaseMovement(const std::string &name, const BT::NodeConfiguration &config) : RosAction(name, config) {}

    std::string ros_name() { return "navigate_to_pose"; }

    void on_send(NavigateToPose::Goal &goal) override;
    void on_feedback(const std::shared_ptr<const NavigateToPose::Feedback> feedback) override;
    void on_result(const rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult &result, const NavigateToPose::Goal &goal) override;

private:
    double last_feedback_time = get_node_handle()->now().seconds();
};