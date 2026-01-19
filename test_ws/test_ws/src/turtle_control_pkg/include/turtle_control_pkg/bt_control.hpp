#pragma once

#include <behaviortree_cpp/action_node.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <chrono>
using namespace std::chrono_literals;

class GoToTarget : public BT::StatefulActionNode
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    GoToTarget(const std::string &name,
               const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("bt_go_to_target_nav2");

        action_client_ =
            rclcpp_action::create_client<NavigateToPose>(
                node_, "navigate_to_pose");
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<double>("x"),
            BT::InputPort<double>("y"),
            BT::InputPort<double>("yaw")
        };
    }

    BT::NodeStatus onStart() override
    {
        if (!action_client_->wait_for_action_server(2s))
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "Nav2 action server not available");
            return BT::NodeStatus::FAILURE;
        }

        double x, y, yaw;
        getInput("x", x);
        getInput("y", y);
        getInput("yaw", yaw);

        NavigateToPose::Goal goal;
        goal.pose = makePose(x, y, yaw);

        auto send_goal_options =
            rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

        send_goal_options.result_callback =
            std::bind(&GoToTarget::resultCallback, this, std::placeholders::_1);

        future_goal_handle_ =
            action_client_->async_send_goal(goal, send_goal_options);

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        rclcpp::spin_some(node_);

        if (result_received_)
        {
            return goal_success_
                ? BT::NodeStatus::SUCCESS
                : BT::NodeStatus::FAILURE;
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        if (goal_handle_)
            action_client_->async_cancel_goal(goal_handle_);

        RCLCPP_WARN(node_->get_logger(), "Navigation cancelled");
    }

private:
    geometry_msgs::msg::PoseStamped makePose(double x, double y, double yaw)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = node_->now();

        pose.pose.position.x = x;
        pose.pose.position.y = y;

        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        pose.pose.orientation = tf2::toMsg(q);

        return pose;
    }

    void resultCallback(const GoalHandleNav::WrappedResult &result)
    {
        result_received_ = true;
        goal_success_ =
            (result.code == rclcpp_action::ResultCode::SUCCEEDED);

        if (goal_success_)
            RCLCPP_INFO(node_->get_logger(), "Nav2 goal reached");
        else
            RCLCPP_ERROR(node_->get_logger(), "Nav2 failed");
    }

    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;

    std::shared_future<GoalHandleNav::SharedPtr> future_goal_handle_;
    GoalHandleNav::SharedPtr goal_handle_;

    bool result_received_ = false;
    bool goal_success_ = false;
};
