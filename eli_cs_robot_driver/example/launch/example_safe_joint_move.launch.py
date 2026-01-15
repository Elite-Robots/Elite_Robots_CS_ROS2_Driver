from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    cpp_or_py = LaunchConfiguration("cpp_or_py")

    cpp_or_py = LaunchConfiguration("cpp_or_py")

    return LaunchDescription(
        [
            Node(
                package="eli_cs_robot_driver",
                executable="publisher_joint_trajectory_controller",
                name="publisher_scaled_joint_trajectory_controller",
                parameters=[position_goals],
                output="screen",
            )
        ]
    )
