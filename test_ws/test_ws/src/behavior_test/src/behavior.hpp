#ifndef BEHAVIOR_HPP
#define BEHAVIOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <string>
#include <memory>

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNav2 = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class GrootNav2Bridge : public rclcpp::Node
{
public:
  GrootNav2Bridge();
  ~GrootNav2Bridge() = default;

private:
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string bt_file_;
  std::string last_target_;
  GoalHandleNav2::SharedPtr current_goal_;

  void check_xml();
  void send_goal(const std::string &target);
};

#endif // BEHAVIOR_HPP