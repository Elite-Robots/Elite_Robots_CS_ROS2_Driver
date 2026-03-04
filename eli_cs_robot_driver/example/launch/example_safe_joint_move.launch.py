from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    cpp_py = LaunchConfiguration('cpp_py').perform(context)

    common_params = {
        'target': LaunchConfiguration('target'),
        'max_vel': LaunchConfiguration('max_vel'),
        'min_time': LaunchConfiguration('min_time'),
        'controller': LaunchConfiguration('controller'),
    }

    if cpp_py == 'py':
        return [
            Node(
                package='eli_cs_robot_driver',
                executable='example_safe_joint_move_py',
                name='example_safe_joint_move_py',
                output='screen',
                parameters=[common_params],
            )
        ]
    else:
        return [
            Node(
                package='eli_cs_robot_driver',
                executable='example_safe_joint_move',
                name='example_safe_joint_move',
                output='screen',
                parameters=[common_params],
            )
        ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'cpp_py',
            default_value='cpp',
            description='Run C++ or Python example node (cpp / py)'
        ),
        DeclareLaunchArgument(
            'target',
            description='Target joint positions (YAML list)'
        ),
        DeclareLaunchArgument(
            'max_vel',
            default_value='0.5',
            description='Maximum joint velocity (rad/s)'
        ),
        DeclareLaunchArgument(
            'min_time',
            default_value='20.0',
            description='Minimum time to reach target'
        ),
        DeclareLaunchArgument(
            'controller',
            default_value='scaled_joint_trajectory_controller',
            description='Controller name'
        ),
        OpaqueFunction(function=launch_setup),
    ])
