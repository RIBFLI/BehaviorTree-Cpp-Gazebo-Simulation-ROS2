
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/action_node.h>
#include <rclcpp/rclcpp.hpp>
#include "turtle_control_pkg/bt_control.hpp"

// extern class GoToTarget;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    BT::BehaviorTreeFactory factory;
    factory.registerNodeType<GoToTarget>("GO_TO_TARGET");

    auto tree = factory.createTreeFromFile("/home/ribfli/test_ws/src/turtle_control_pkg/config/behavior.xml");

    auto status = tree.tickOnce();

    while (status == BT::NodeStatus::RUNNING)
    {
        tree.sleep(std::chrono::milliseconds(100));
        status = tree.tickOnce();
    }

    rclcpp::shutdown();
    return 0;
}