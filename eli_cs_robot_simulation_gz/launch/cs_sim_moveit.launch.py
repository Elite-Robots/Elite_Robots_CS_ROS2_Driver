# Author: Chen Shichao

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    # Initialize Arguments
    cs_type = LaunchConfiguration("cs_type")
    safety_limits = LaunchConfiguration("safety_limits")
    # General arguments
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    moveit_config_file = LaunchConfiguration("moveit_config_file")
    moveit_joint_limits_file = LaunchConfiguration("moveit_joint_limits_file")
    moveit_controllers_file = LaunchConfiguration("moveit_controllers_file")
    prefix = LaunchConfiguration("prefix")

    cs_type_value = cs_type.perform(context)
    controllers_file_value = controllers_file.perform(context)
    description_file_value = description_file.perform(context)
    moveit_config_file_value = moveit_config_file.perform(context)
    moveit_joint_limits_file_value = moveit_joint_limits_file.perform(context)
    moveit_controllers_file_value = moveit_controllers_file.perform(context)
    if cs_type_value.endswith("h"):
        controllers_file_value = "cs_controllers_5f.yaml"
        description_file_value = "cs.urdf.xacro"
        moveit_config_file_value = "cs_5f.srdf.xacro"
        moveit_joint_limits_file_value = "joint_limits_5f.yaml"
        moveit_controllers_file_value = "controllers_5f.yaml"

    cs_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("eli_cs_robot_simulation_gz"), "/launch", "/cs_sim_control.launch.py"]
        ),
        launch_arguments={
            "cs_type": cs_type,
            "safety_limits": safety_limits,
            "runtime_config_package": runtime_config_package,
            "controllers_file": controllers_file_value,
            "description_package": description_package,
            "description_file": description_file_value,
            "prefix": prefix,
            "launch_rviz": "false",
        }.items(),
    )

    cs_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("eli_cs_robot_moveit_config"), "/launch", "/cs_moveit.launch.py"]
        ),
        launch_arguments={
            "cs_type": cs_type,
            "safety_limits": safety_limits,
            "description_package": description_package,
            "description_file": description_file_value,
            "moveit_config_package": moveit_config_package,
            "moveit_config_file": moveit_config_file_value,
            "moveit_joint_limits_file": moveit_joint_limits_file_value,
            "moveit_controllers_file": moveit_controllers_file_value,
            "prefix": prefix,
            "use_sim_time": "true",
            "launch_rviz": "true",
        }.items(),
    )

    nodes_to_launch = [
        cs_control_launch,
        cs_moveit_launch,
    ]

    return nodes_to_launch


def generate_launch_description():
    declared_arguments = []
    # CS specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "cs_type",
            description="Type/series of used ELITE CS robot.",
            choices=["cs63", "cs66", "cs612", "cs616", "cs620", "cs625", "cs66a", "cs68", "cs520h"],
            default_value="cs66",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        )
    )
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="eli_cs_robot_simulation_gz",
            description='Package with the controller\'s configuration in "config" folder. \
        Usually the argument is not set, it enables use of a custom setup.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="cs_controllers.yaml",
            description="YAML file with the controllers configuration. Overridden for 5-axis cs_type (suffix 'h').",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="eli_cs_robot_description",
            description="Description package with robot URDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="cs.urdf.xacro",
            description="URDF/XACRO description file with the robot. Overridden for 5-axis cs_type (suffix 'h').",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value="eli_cs_robot_moveit_config",
            description="MoveIt config package with robot SRDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom moveit config.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_file",
            default_value="cs.srdf.xacro",
            description="MoveIt SRDF/XACRO description file with the robot. Overridden for 5-axis cs_type (suffix 'h').",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_joint_limits_file",
            default_value="joint_limits.yaml",
            description="MoveIt joint limits file. Overridden for 5-axis cs_type (suffix 'h').",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_controllers_file",
            default_value="controllers.yaml",
            description="MoveIt controller list configuration file. Overridden for 5-axis cs_type (suffix 'h').",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
