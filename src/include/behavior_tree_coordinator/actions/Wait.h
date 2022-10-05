/** *******************************************************
 * PeTRA - University of Applied Sciences Karlsruhe
 * Module : ROS2-Node "neobotixCoordinator"
 * Purpose : Wait in seconds
 *
 * @author Andreas Zachariae
 * @since 1.1.0 (2021.09.06)
 *********************************************************/
#pragma once

#include <behavior_tree_coordinator/default.h>

#include <behaviortree_ros/components/RosNode.h>
#include <behaviortree_ros/tools/convert.h>

#include <time.h>

class Wait : public RosNode
{
public:
    static BT::PortsList providedPorts() { return {
        BT::InputPort<int>("seconds"),
    }; }

    Wait(const std::string &name, const BT::NodeConfiguration &config) : RosNode(name, config) {}

    std::string ros_name() { return "Wait"; }

    BT::NodeStatus on_start() override;
    BT::NodeStatus on_running() override;
    void on_halted() override;

private:
    time_t start_time_;
    time_t duration_;
    double last_feedback_time = get_node_handle()->now().seconds();
};