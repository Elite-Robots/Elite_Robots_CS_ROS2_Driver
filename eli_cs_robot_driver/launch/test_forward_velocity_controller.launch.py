from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    velocity_goals = PathJoinSubstitution(
        [FindPackageShare("eli_cs_robot_driver"),"example", "config", "test_velocity_goal_publishers_config.yaml"]
    )

    return LaunchDescription(
        [
            Node(
                package="ros2_controllers_test_nodes",
                executable="publisher_forward_position_controller",
                name="publisher_forward_velocity_controller",
                parameters=[velocity_goals],
                output="screen",
            )
        ]
    )
