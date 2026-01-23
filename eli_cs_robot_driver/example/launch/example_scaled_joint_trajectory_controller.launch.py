from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _launch_setup(context):
    robot_model = LaunchConfiguration("robot_model").perform(context)

    if robot_model == "cs66a":
        config_name = "test_goal_publishers_config_cs66a.yaml"
    else:
        config_name = "test_goal_publishers_config.yaml"

    position_goals = PathJoinSubstitution(
        [
            FindPackageShare("eli_cs_robot_driver"),
            config_name,
        ]
    )

    return [
        Node(
            package="ros2_controllers_test_nodes",
            executable="publisher_joint_trajectory_controller",
            name="publisher_scaled_joint_trajectory_controller",
            parameters=[position_goals],
            output="screen",
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_model", default_value=""),
            OpaqueFunction(function=_launch_setup),
        ]
    )
