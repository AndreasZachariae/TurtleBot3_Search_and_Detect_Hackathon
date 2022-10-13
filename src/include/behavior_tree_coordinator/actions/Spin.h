/** *******************************************************
 * PeTRA - University of Applied Sciences Karlsruhe
 * Module : ROS2-Node "neobotixCoordinator"
 * Purpose : Provides the ROS2-Action client "SpinToYaw"
 *
 * @author Andreas Zachariae
 * @author Moritz Weisenböhler
 * @since 1.0.0 (2020.08.26)
 *********************************************************/
#pragma once

#include <behavior_tree_coordinator/default.h>

#include <behaviortree_ros/components/RosAction.h>
#include <behaviortree_ros/tools/convert.h>
#include <nav2_msgs/action/spin.hpp>

using SpinToYaw = nav2_msgs::action::Spin;

class Spin : public RosAction<SpinToYaw>
{
public:
    static BT::PortsList providedPorts() { return {BT::InputPort<float>("target_yaw")}; }

    Spin(const std::string &name, const BT::NodeConfiguration &config) : RosAction(name, config) {}

    std::string ros_name() { return "spin"; }

    void on_send(SpinToYaw::Goal &goal) override;
    void on_feedback(const std::shared_ptr<const SpinToYaw::Feedback> feedback) override;
    void on_result(const rclcpp_action::ClientGoalHandle<SpinToYaw>::WrappedResult &result, const SpinToYaw::Goal &goal) override;

private:
    double last_feedback_time = get_node_handle()->now().seconds();
};