#include <behavior_tree_coordinator/conditions/IsBallFound.h>

IsBallFound::IsBallFound(const std::string &name, const BT::NodeConfiguration &config) : RosCondition(name, config)
{
    ball_pose_subscription_ = get_node_handle()->create_subscription<geometry_msgs::msg::Pose>("ball_pose", 10, [&](const geometry_msgs::msg::Pose::SharedPtr msg)
                                                                                               { ball_pose_ = msg; });
}

BT::NodeStatus IsBallFound::on_check()
{
    bool trigger = false;
    BT::Optional<bool> trigger_input = getInput<bool>("next_goal_trigger");
    if (trigger_input.has_value())
    {
        if (trigger_input.value() == true)
            trigger = true;
    }

    if (trigger == true)
    {
        log("Next goal triggered, resetting ball_pose_msg");
    }

    if (!ball_pose_->position.x == 0.0)
    {
        setOutput<float>("ball_x", ball_pose_->position.x);
        setOutput<float>("ball_y", ball_pose_->position.y);
        setOutput<float>("ball_quaternion_x", ball_pose_->orientation.x);
        setOutput<float>("ball_quaternion_y", ball_pose_->orientation.y);
        setOutput<float>("ball_quaternion_z", ball_pose_->orientation.z);
        setOutput<float>("ball_quaternion_w", ball_pose_->orientation.w);

        log("Ball Found at location: (x=" + convert::ftos(ball_pose_->position.x) + ", y=" + convert::ftos(ball_pose_->position.y) + ")");

        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
}