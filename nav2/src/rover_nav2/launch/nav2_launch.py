from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    package_dir = os.path.dirname(os.path.realpath(__file__))

    return LaunchDescription([
        DeclareLaunchArgument(
            'slam',
            default_value='true',
            description='Whether to run SLAM'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(package_dir, '..', 'params', 'nav2_params.yaml'),
            description='Full path to the ROS2 parameters file to use'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Real or simulated time (e.g.: Isaac Sim or rosbag)'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([FindPackageShare('nav2_bringup'), '/launch/navigation_launch.py']),
            launch_arguments={
                'slam': LaunchConfiguration('slam'),
                'params_file': LaunchConfiguration('params_file'),
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }.items()
        )
    ])