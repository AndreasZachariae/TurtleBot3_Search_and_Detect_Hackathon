// Copyright 2019 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <inttypes.h>
#include <memory>
#include <string>
#include <iostream>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class MinimalNavAction : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    explicit MinimalNavAction(const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions())
        : Node("minimal_nav_action", node_options), goal_done_(false)
    {
        this->client_ptr_ = rclcpp_action::create_client<NavigateToPose>(
            this->get_node_base_interface(),
            this->get_node_graph_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(),
            "navigate_to_pose");

        this->timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&MinimalNavAction::send_goal, this));
    }

    bool is_goal_done() const
    {
        return this->goal_done_;
    }

    void send_goal()
    {
        using namespace std::placeholders;

        this->timer_->cancel();

        this->goal_done_ = false;

        if (!this->client_ptr_)
        {
            RCLCPP_ERROR(this->get_logger(), "Action client not initialized");
        }

        if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10)))
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            this->goal_done_ = true;
            return;
        }

        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose.pose.position.x = 2;
        goal_msg.pose.pose.position.y = 0;
        goal_msg.pose.pose.position.z = 0;
        goal_msg.pose.pose.orientation.w = 1;
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->now();

        RCLCPP_INFO(this->get_logger(), "Sending goal");

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&MinimalNavAction::goal_response_callback, this, _1);
        send_goal_options.feedback_callback =
            std::bind(&MinimalNavAction::feedback_callback, this, _1, _2);
        send_goal_options.result_callback =
            std::bind(&MinimalNavAction::result_callback, this, _1);
        auto goal_handle_future = this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool goal_done_;

    void goal_response_callback(std::shared_future<GoalHandleNavigateToPose::SharedPtr> future)
    {
        auto goal_handle = future.get();
        if (!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void feedback_callback(
        GoalHandleNavigateToPose::SharedPtr,
        const std::shared_ptr<const NavigateToPose::Feedback> feedback)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "Current position: (x=" + std::to_string(feedback->current_pose.pose.position.x) +
                ", y=" + std::to_string(feedback->current_pose.pose.position.y) +
                "), Time elapsed: " + std::to_string(feedback->navigation_time.sec) + "s");
    }

    void result_callback(const GoalHandleNavigateToPose::WrappedResult &result)
    {
        this->goal_done_ = true;
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            return;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Goal reached!");
    }
}; // class MinimalNavAction

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto action_client = std::make_shared<MinimalNavAction>();

    while (!action_client->is_goal_done())
    {
        rclcpp::spin_some(action_client);
    }

    rclcpp::shutdown();
    return 0;
}