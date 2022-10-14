/** *******************************************************
 * PeTRA - University of Applied Sciences Karlsruhe
 * Module : ROS2-Node "PeTRACoordinator"
 * Purpose : Calculates BoxPickPosition from tf transformations to apriltag no. 4
 *
 * @author Andreas Zachariae
 * @author Frederik Plahl
 * @since 1.1.0 (2021.09.06)
 *********************************************************/
#pragma once

#include <behavior_tree_coordinator/default.h>

#include <behaviortree_ros/components/RosNode.h>
#include <behaviortree_ros/tools/convert.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <time.h>
#include <thread>

class GetBallPose : public RosNode
{
public:
    static BT::PortsList providedPorts() { return {BT::InputPort<std::string>("marker_frame"),
                                                   BT::InputPort<int>("max_seconds"),
                                                   BT::OutputPort<float>("ball_x"),
                                                   BT::OutputPort<float>("ball_y"),
                                                   BT::OutputPort<float>("ball_z"),
                                                   BT::OutputPort<float>("ball_quaternion_x"),
                                                   BT::OutputPort<float>("ball_quaternion_y"),
                                                   BT::OutputPort<float>("ball_quaternion_z"),
                                                   BT::OutputPort<float>("ball_quaternion_w")}; }

    GetBallPose(const std::string &name, const BT::NodeConfiguration &config);

    std::string ros_name() { return "GetBallPose"; }

    BT::NodeStatus on_start() override;
    BT::NodeStatus on_running() override;
    void on_halted() override {}

private:
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
    time_t start_time_;
    time_t duration_;
    time_t last_checked_;
    std::string marker_frame_;
};