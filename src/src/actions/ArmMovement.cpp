#include <behavior_tree_coordinator/actions/ArmMovement.h>

void ArmMovement::on_send(MoveArm::Goal &goal)
{
    // Check if Optional is valid. If not, throw its error
    BT::Optional<float> x_input = getInput<float>("x_mm");
    BT::Optional<float> y_input = getInput<float>("y_mm");
    BT::Optional<float> z_input = getInput<float>("z_mm");
    BT::Optional<float> gripper_input = getInput<float>("gripper_position");

    if (!x_input.has_value())
    {
        throw BT::RuntimeError("missing required input [x]: ", x_input.error());
    }
    if (!y_input.has_value())
    {
        throw BT::RuntimeError("missing required input [y]: ", y_input.error());
    }
    if (!z_input.has_value())
    {
        throw BT::RuntimeError("missing required input [z]: ", z_input.error());
    }
    if (!gripper_input.has_value())
    {
        throw BT::RuntimeError("missing required input [gripper_position]: ", gripper_input.error());
    }

    goal.pose.pose.position.x = x_input.value();
    goal.pose.pose.position.y = y_input.value();
    goal.pose.pose.position.z = z_input.value();
    goal.gripper_position = gripper_input.value();

    goal.pose.pose.orientation.w = 1; // hard-coded orientation (x=0, y=0, z=0, w=1)
    goal.pose.header.stamp = get_node_handle()->now();

    log("Goal: (x=" + convert::ftos(goal.pose.pose.position.x) + ", y=" + convert::ftos(goal.pose.pose.position.y) + ", z=" + convert::ftos(goal.pose.pose.position.z) + ")");
}

void ArmMovement::on_feedback(const std::shared_ptr<const MoveArm::Feedback> feedback)
{
    log("Position: (x=" + convert::ftos(feedback->current_pose.pose.position.x) +
        ", y=" + convert::ftos(feedback->current_pose.pose.position.y) +
        ", z=" + convert::ftos(feedback->current_pose.pose.position.z) +
        "), Progress: " + convert::ftos(feedback->progress * 100) +
        "%, Time: " + std::to_string(feedback->movement_time.sec) + "s");
}

void ArmMovement::on_result(const rclcpp_action::ClientGoalHandle<MoveArm>::WrappedResult &, const MoveArm::Goal &goal)
{
    log("Goal reached! (x=" + convert::ftos(goal.pose.pose.position.x) +
        ", y=" + convert::ftos(goal.pose.pose.position.y) +
        ", z=" + convert::ftos(goal.pose.pose.position.z) +
        "), Total time: " + std::to_string((int)(get_node_handle()->now().seconds() - -goal.pose.header.stamp.sec)) + "s");
}