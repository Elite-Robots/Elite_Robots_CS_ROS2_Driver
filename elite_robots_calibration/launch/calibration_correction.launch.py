# Author: Lovro Ivanov

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip", description="The IP address at which the robot is reachable."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "target_filename",
            default_value="robot_calibration.yaml",
            description="The extracted calibration information "
            "will be written to this target file.",
        )
    )

    # Initialize Arguments
    robot_ip = LaunchConfiguration("robot_ip")
    output_filename = LaunchConfiguration("target_filename")

    calibration_correction = Node(
        package="elite_robots_calibration",
        executable="calibration_correction",
        parameters=[{"robot_ip": robot_ip}, {"output_filename": output_filename}],
        output="screen",
    )

    nodes_to_start = [
        calibration_correction,
    ]

    return LaunchDescription(declared_arguments + nodes_to_start)
