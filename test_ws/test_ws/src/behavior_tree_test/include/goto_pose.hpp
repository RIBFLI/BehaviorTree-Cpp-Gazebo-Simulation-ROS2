#ifndef GOTOPOSE_HPP
#define GOTOPOSE_HPP

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

// Struct to keep location pose data for mobile robot (2D + yaw)
struct Pose2D
{
  double x;
  double y;
  double yaw;  // rotation in radians
};

// Template specialization to convert a string to Pose2D
namespace BT
{
  template <>
  inline Pose2D convertFromString(StringView str)
  {
    // We expect real numbers separated by semicolons: x;y;yaw
    auto parts = splitString(str, ';');
    if (parts.size() != 3)
    {
      throw RuntimeError("invalid Pose2D format. Expected x;y;yaw");
    }
    else
    {
      Pose2D output;
      output.x = convertFromString<double>(parts[0]);
      output.y = convertFromString<double>(parts[1]);
      output.yaw = convertFromString<double>(parts[2]);
      return output;
    }
  }
} // end namespace BT

class GoToPose : public BT::StatefulActionNode
{
private:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNav = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  
  rclcpp::Node::SharedPtr node_ptr_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_ptr_;
  GoalHandleNav::SharedPtr goal_handle_;
  rclcpp_action::ResultCode navigation_result_;
  bool done_flag_;
  
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
  
  void goal_response_callback(const GoalHandleNav::SharedPtr &goal_handle);
  void feedback_callback(GoalHandleNav::SharedPtr,
                        const std::shared_ptr<const NavigateToPose::Feedback> feedback);
  void result_callback(const GoalHandleNav::WrappedResult &result);

public:
  GoToPose(const std::string &name,
           const BT::NodeConfig &config,
           rclcpp::Node::SharedPtr node_ptr)
      : BT::StatefulActionNode(name, config),
        node_ptr_(node_ptr),
        done_flag_(false),
        navigation_result_(rclcpp_action::ResultCode::UNKNOWN)
  {
    client_ptr_ = rclcpp_action::create_client<NavigateToPose>(node_ptr_, "navigate_to_pose");
  }

  static BT::PortsList providedPorts()
  {
    return {
        BT::InputPort<Pose2D>("target", "Target pose (x;y;yaw)"),
        BT::InputPort<std::string>("frame_id", "map", "Reference frame")
    };
  }
};

#endif // GOTOPOSE_HPP
