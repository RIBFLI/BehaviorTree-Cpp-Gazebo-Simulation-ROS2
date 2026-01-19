#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

class GoToTarget : public BT::StatefulActionNode
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

    GoToTarget(const std::string &name, const BT::NodeConfiguration &config)
        : BT::StatefulActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("bt_go_to_target");
        client_ = rclcpp_action::create_client<NavigateToPose>(node_, "navigate_to_pose");
    }

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<std::string>("target")};
    }

    BT::NodeStatus onStart() override
    {
        if (!client_->wait_for_action_server(5s))
        {
            RCLCPP_ERROR(node_->get_logger(), "Nav2 action server not available");
            return BT::NodeStatus::FAILURE;
        }

        std::string target;
        getInput("target", target);

        double x, y, yaw;
        sscanf(target.c_str(), "%lf;%lf;%lf", &x, &y, &yaw);

        NavigateToPose::Goal goal;
        goal.pose.header.frame_id = "map";
        goal.pose.header.stamp = node_->now();
        goal.pose.pose.position.x = x;
        goal.pose.pose.position.y = y;

        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        goal.pose.pose.orientation = tf2::toMsg(q);

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

        auto future_goal = client_->async_send_goal(goal, send_goal_options);

        if (rclcpp::spin_until_future_complete(node_, future_goal) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            return BT::NodeStatus::FAILURE;
        }

        goal_handle_ = future_goal.get();
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        auto result_future = client_->async_get_result(goal_handle_);

        if (rclcpp::spin_until_future_complete(node_, result_future, 10ms) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto result = result_future.get();
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
                return BT::NodeStatus::SUCCESS;
            else
                return BT::NodeStatus::FAILURE;
        }

        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        RCLCPP_WARN(node_->get_logger(), "GO_TO_TARGET halted");
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
    GoalHandle::SharedPtr goal_handle_;
};
