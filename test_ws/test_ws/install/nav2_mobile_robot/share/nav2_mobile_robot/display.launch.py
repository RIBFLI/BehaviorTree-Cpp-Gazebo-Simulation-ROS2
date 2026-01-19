from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ===== Launch argument =====
    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true"
    )

    # ===== Robot description =====
    xacro_path = "nav2_mobile_robot.xacro"

    robot_description = PathJoinSubstitution(
        [get_package_share_directory("nav2_mobile_robot"), xacro_path]
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": Command(["xacro ", robot_description]),
                "use_sim_time": use_sim_time,
            }
        ],
    )

    # ===== RViz =====
    rviz_file = "rviz.rviz"
    rviz_path = PathJoinSubstitution(
        [get_package_share_directory("nav2_mobile_robot"), rviz_file]
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_path],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            robot_state_publisher_node,
            rviz2_node,
        ]
    )
