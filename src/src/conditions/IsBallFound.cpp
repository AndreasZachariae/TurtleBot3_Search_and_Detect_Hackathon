#include <behavior_tree_coordinator/conditions/IsBallFound.h>

IsBallFound::IsBallFound(const std::string &name, const BT::NodeConfiguration &config) : RosCondition(name, config)
{
    ball_found_subscription_ = get_node_handle()->create_subscription<std_msgs::msg::Bool>("ball_found", 10, [&](const std_msgs::msg::Bool::SharedPtr msg)
                                                                                           { if (msg->data) 
                                                                                          {is_ball_found = true;} });
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
        is_ball_found = false;
        log("Next goal triggered, resetting ball_found");
    }

    if (is_ball_found)
    {

        log("Ball Found");

        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
}