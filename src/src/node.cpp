/** *******************************************************
 * PeTRA - University of Applied Sciences Karlsruhe
 * Module : ROS2-Node "Coordinator"
 * Purpose : main-function and start of the behavior tree
 *
 * @author Moritz Weisenböhler
 * @author Andreas Zachariae
 * @since 1.0.0 (2020.08.26)
 *********************************************************/
#include <behaviortree_cpp_v3/bt_factory.h>

#include <behaviortree_ros/components/RosInterface.h>

#include <behavior_tree_coordinator/actions/BaseMovement.h>
#include <behavior_tree_coordinator/actions/ArmMovement.h>
#include <behavior_tree_coordinator/actions/BatteryCharging.h>
#include <behavior_tree_coordinator/actions/Wait.h>
#include <behavior_tree_coordinator/conditions/CheckStop.h>
#include <behavior_tree_coordinator/conditions/CheckBattery.h>
#include <behavior_tree_coordinator/conditions/IsBallFound.h>
#include <behavior_tree_coordinator/conditions/IsGoalDone.h>
#include <behavior_tree_coordinator/actions/Spin.h>
#include <behavior_tree_coordinator/actions/GetBallPose.h>

BT::Tree create_tree(const std::string &path)
{
    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<CheckStop>("CheckStop");
    factory.registerNodeType<CheckBattery>("CheckBattery");
    factory.registerNodeType<BaseMovement>("BaseMovement");
    factory.registerNodeType<ArmMovement>("ArmMovement");
    factory.registerNodeType<BatteryCharging>("BatteryCharging");
    factory.registerNodeType<Wait>("Wait");
    factory.registerNodeType<IsBallFound>("IsBallFound");
    factory.registerNodeType<IsGoalDone>("IsGoalDone");
    factory.registerNodeType<Spin>("Spin");
    factory.registerNodeType<GetBallPose>("GetBallPose");

    return factory.createTreeFromFile(path);
}

BT::NodeStatus run_tree(BT::Tree &tree)
{
    BT::NodeStatus status;
    BT::NodeStatus last_status;

    do
    {
        RosInterface::spin_some();

        status = tree.tickRoot();

        if (status != last_status)
        {
            switch (status)
            {
            case BT::NodeStatus::IDLE:
                std::cout << "[MainTree] Status: IDLE" << std::endl;
                break;
            case BT::NodeStatus::RUNNING:
                std::cout << "[MainTree] Status: RUNNING" << std::endl;
                break;
            case BT::NodeStatus::SUCCESS:
                std::cout << "[MainTree] Status: SUCCESS" << std::endl;
                break;
            case BT::NodeStatus::FAILURE:
                std::cout << "[MainTree] Status: FAILURE" << std::endl;
                break;
            }

            last_status = status;
        }

    } while (status == BT::NodeStatus::IDLE || status == BT::NodeStatus::RUNNING);

    return status;
}

int main(int argc, char **argv)
{
    RosInterface::init("Coordinator", argc, argv);

    RosInterface::get_node_handle()->declare_parameter("main_tree_path");

    std::string main_tree_path;

    try
    {
        main_tree_path = RosInterface::get_node_handle()->get_parameter("main_tree_path").as_string();
        std::cout << main_tree_path << std::endl;
    }
    catch (const rclcpp::exceptions::ParameterNotDeclaredException &e)
    {
        std::cerr << e.what() << " not declared\n";
    }

    BT::Tree tree = create_tree(main_tree_path);

    run_tree(tree);

    return 0;
}
