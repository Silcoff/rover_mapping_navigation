from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Also commented out from Norberts github, MARKED irrelevant
    # Paths to configuration files
    # traversability_filter_chain = os.path.join(
    #     get_package_share_directory('ros2_traversability_mapping'),
    #     'config',
    #     'ros2_traversability_mapping_filter_chain.yaml'
    # )
    # traversability_params = os.path.join(
    #     get_package_share_directory('ros2_traversability_mapping'),
    #     'config',
    #     'ros2_traversability_mapping.yaml'
    # )

    # return LaunchDescription([
    #     # Traversability Mapping Node
    #     Node(
    #         package='ros2_traversability_mapping',
    #         executable='ros2_traversability_mapping_node',
    #         name='ros2_traversability_mapping',
    #         output='screen',
    #         parameters=[
    #             {'input_topic': '/rtabmap/elevation_map'},
    #             {'output_topic': 'traversability_map'},
    #             # traversability_filter_chain,
    #             # traversability_params
    #         ]
    #     )
    # ])
