/** *******************************************************
 * PeTRA - University of Applied Sciences Karlsruhe
 * Module : ROS2-Node "neobotixCoordinator"
 * Purpose : Provides the ROS2-Action client "MoveArm"
 *
 * @author Andreas Zachariae
 * @author Moritz Weisenböhler
 * @since 1.0.0 (2020.08.26)
 *********************************************************/
#pragma once

#include <behavior_tree_coordinator/default.h>

#include <behaviortree_ros/components/RosAction.h>
#include <behaviortree_ros/tools/convert.h>
#include <petra_interfaces/action/move_arm.hpp>

using MoveArm = petra_interfaces::action::MoveArm;

class ArmMovement : public RosAction<MoveArm>
{
public:
    static BT::PortsList providedPorts() { return {BT::InputPort<float>("x_mm"),
                                                   BT::InputPort<float>("y_mm"),
                                                   BT::InputPort<float>("z_mm"),
                                                   BT::InputPort<float>("gripper_position")}; }

    ArmMovement(const std::string &name, const BT::NodeConfiguration &config) : RosAction(name, config) {}

    std::string ros_name() { return "MoveArm"; }

    void on_send(MoveArm::Goal &goal) override;
    void on_feedback(const std::shared_ptr<const MoveArm::Feedback> feedback) override;
    void on_result(const rclcpp_action::ClientGoalHandle<MoveArm>::WrappedResult &result, const MoveArm::Goal &goal) override;
};