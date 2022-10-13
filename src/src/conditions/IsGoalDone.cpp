#include <behavior_tree_coordinator/conditions/IsGoalDone.h>

IsGoalDone::IsGoalDone(const std::string &name, const BT::NodeConfiguration &config) : RosCondition(name, config)
{
    goal_finished_subscription_ = get_node_handle()->create_subscription<std_msgs::msg::String>("/search_and_detect/finished", 10, [&](const std_msgs::msg::String::SharedPtr msg)
                                                                                                { new_goal_finished_msg_ = true; });
}

BT::NodeStatus IsGoalDone::on_check()
{
    if (new_goal_finished_msg_ == true)
    {
        setOutput<bool>("next_goal_trigger", true);
        new_goal_finished_msg_ = false;

        log("Goal reached, triggering next goal...");

        return BT::NodeStatus::SUCCESS;
    }

    setOutput<bool>("next_goal_trigger", false);

    return BT::NodeStatus::FAILURE;
}