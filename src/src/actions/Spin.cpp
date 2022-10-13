#include <behavior_tree_coordinator/actions/Spin.h>

void Spin::on_send(SpinToYaw::Goal &goal)
{
    // Check if Optional is valid. If not, throw its error
    BT::Optional<float> yaw_input = getInput<float>("target_yaw");

    goal.target_yaw = yaw_input.value();

    log("Start Spinning " + convert::ftos(yaw_input.value()) + " radians");
}

void Spin::on_feedback(const std::shared_ptr<const SpinToYaw::Feedback> feedback)
{
    if ((get_node_handle()->now().seconds() - last_feedback_time) >= 1)
    {
        log("still spinning...");

        last_feedback_time = get_node_handle()->now().seconds();
    }
}

void Spin::on_result(const rclcpp_action::ClientGoalHandle<SpinToYaw>::WrappedResult &, const SpinToYaw::Goal &goal)
{
    log("Finished Spin");
}