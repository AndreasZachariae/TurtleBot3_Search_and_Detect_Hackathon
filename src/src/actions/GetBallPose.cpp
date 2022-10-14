#include <behavior_tree_coordinator/actions/GetBallPose.h>

using namespace std::chrono_literals;

GetBallPose::GetBallPose(const std::string &name, const BT::NodeConfiguration &config) : RosNode(name, config)
{
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_node_handle()->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

BT::NodeStatus GetBallPose::on_start()
{
    BT::Optional<std::string> marker_frame = getInput<std::string>("marker_frame");
    BT::Optional<int> max_seconds = getInput<int>("max_seconds");

    if (!marker_frame.has_value())
    {
        throw BT::RuntimeError("missing required input [marker_frame]: ", marker_frame.error());
    }
    if (!max_seconds.has_value())
    {
        throw BT::RuntimeError("missing required input [max_seconds]: ", max_seconds.error());
    }

    duration_ = max_seconds.value();
    start_time_ = time(NULL);
    last_checked_ = time(NULL);
    marker_frame_ = marker_frame.value();

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GetBallPose::on_running()
{
    if ((time(NULL) - start_time_) < duration_)
    {
        if ((time(NULL) - last_checked_) > 1)
        {
            geometry_msgs::msg::TransformStamped transformStamped;
            last_checked_ = time(NULL);

            try
            {
                // tf_buffer_->canTransform("base_link", marker_frame_);
                transformStamped = tf_buffer_->lookupTransform(
                    "map", marker_frame_, get_node_handle()->get_clock()->now(), 3s);
                // tf2::TimePointZero);
            }
            catch (tf2::TransformException &ex)
            {
                RCLCPP_INFO(
                    get_node_handle()->get_logger(), "Could not transform: %s", ex.what());

                // log("Could not transform!" + ex.what());
                return BT::NodeStatus::RUNNING;
            }

            setOutput<float>("ball_x", transformStamped.transform.translation.x);
            setOutput<float>("ball_y", transformStamped.transform.translation.y);
            setOutput<float>("ball_z", transformStamped.transform.translation.z);
            setOutput<float>("ball_quaternion_x", transformStamped.transform.rotation.x);
            setOutput<float>("ball_quaternion_y", transformStamped.transform.rotation.y);
            setOutput<float>("ball_quaternion_z", transformStamped.transform.rotation.z);
            setOutput<float>("ball_quaternion_z", transformStamped.transform.rotation.w);

            log("Ball Pose: (x=" + convert::ftos(transformStamped.transform.translation.x) + ", y=" + convert::ftos(transformStamped.transform.translation.y) + ", z=" + convert::ftos(transformStamped.transform.translation.z) + ")");

            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            return BT::NodeStatus::RUNNING;
        }
    }

    return BT::NodeStatus::FAILURE;
}