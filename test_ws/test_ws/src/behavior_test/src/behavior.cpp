#include "behavior.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tinyxml2.h>

GrootNav2Bridge::GrootNav2Bridge() : Node("groot_nav2_bridge")
{
  client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

  bt_file_ = "/home/ribfli/test_ws/src/nav2_mobile_robot/src/nav2_mobile_robot/nav2_mobile_robot/behavior.xml";

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&GrootNav2Bridge::check_xml, this));

  RCLCPP_INFO(get_logger(), "Groot → Nav2 bridge started");
}

void GrootNav2Bridge::check_xml()
{
  tinyxml2::XMLDocument doc;
  if (doc.LoadFile(bt_file_.c_str()) != tinyxml2::XML_SUCCESS)
    return;

  auto* root = doc.RootElement();
  if (!root) return;

  auto* node = root->FirstChildElement("BehaviorTree");
  if (!node) return;

  auto* action = node->FirstChildElement("GO_TO_TARGET");
  if (!action) return;

  const char* target = action->Attribute("target");
  if (!target) return;

  if (last_target_ != target)
  {
    last_target_ = target;
    send_goal(target);
  }
}

void GrootNav2Bridge::send_goal(const std::string &target)
{
  if (!client_->wait_for_action_server(std::chrono::seconds(2)))
  {
    RCLCPP_ERROR(get_logger(), "Nav2 action server not available");
    return;
  }

  if (current_goal_)
  {
    client_->async_cancel_goal(current_goal_);
  }

  double x, y, yaw;
  sscanf(target.c_str(), "%lf;%lf;%lf", &x, &y, &yaw);

  NavigateToPose::Goal goal;
  goal.pose.header.frame_id = "map";
  goal.pose.header.stamp = this->now();
  goal.pose.pose.position.x = x;
  goal.pose.pose.position.y = y;

  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  goal.pose.pose.orientation.x = q.x();
  goal.pose.pose.orientation.y = q.y();
  goal.pose.pose.orientation.z = q.z();
  goal.pose.pose.orientation.w = q.w();

  RCLCPP_INFO(get_logger(), "Sending goal: %.2f %.2f %.2f", x, y, yaw);

  auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  options.feedback_callback =
    [](GoalHandleNav2::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback)
    {
      RCLCPP_INFO(rclcpp::get_logger("groot_nav2"), "Remaining: %.2f m", feedback->distance_remaining);
    };

  options.result_callback =
    [this](const GoalHandleNav2::WrappedResult &)
    {
      RCLCPP_INFO(this->get_logger(), "Goal finished");
      current_goal_.reset();
    };

  options.goal_response_callback =
    [this](const GoalHandleNav2::SharedPtr & goal_handle)
    {
      if (!goal_handle)
      {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
      }
      else
      {
        current_goal_ = goal_handle;
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server");
      }
    };

  client_->async_send_goal(goal, options);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GrootNav2Bridge>());
  rclcpp::shutdown();
  return 0;
}