#include <behavior_tree_coordinator/conditions/CheckStop.h>

CheckStop::CheckStop(const std::string &name, const BT::NodeConfiguration &config) : RosCondition(name, config)
{
    stop_subscription_ = get_node_handle()->create_subscription<std_msgs::msg::Empty>("Stop", 10, [&](const std_msgs::msg::Empty::SharedPtr)
                                                                                      { stop_recieved_ = true; });
}

BT::NodeStatus CheckStop::on_check()
{
    if (stop_recieved_)
    {
        log("Stop recieved");

        return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::SUCCESS;
}