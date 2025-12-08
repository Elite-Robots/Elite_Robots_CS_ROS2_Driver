from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    position_goals = PathJoinSubstitution(
        [FindPackageShare("eli_cs_robot_driver"), "example","config", "test_goal_publishers_config.yaml"]
    )

    return LaunchDescription(
        [
            Node(
                package="ros2_controllers_test_nodes",
                executable="publisher_joint_trajectory_controller",
                name="publisher_scaled_joint_trajectory_controller",
                parameters=[position_goals],
                output="screen",
            )
        ]
    )
