from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Paths to ZED configuration files
    zed_config_common = os.path.join(
        get_package_share_directory('ros2_traversability_mapping'),
        'config',
        'common_stereo.yaml'
    )
    zed_config_specific = os.path.join(
        get_package_share_directory('ros2_traversability_mapping'),
        'config',
        'zed2i.yaml'
    )

    return LaunchDescription([
        # RTAB-Map Node
        Node(
            package='rtabmap_ros',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                'args': '--delete_db_on_start --Optimizer/GravitySigma 0.3 --Odom/Strategy 1 --Vis/CorType 1 --OdomF2M/MaxSize 1000 --Vis/MaxFeatures 600',
                'depth_topic': '/zed/zed_node/depth/depth_registered',
                'rgb_topic': '/zed/zed_node/rgb/image_rect_color',
                'camera_info_topic': '/zed/zed_node/rgb/camera_info',
                'approx_sync': False,
                'wait_imu_to_init': True,
                'imu_topic': '/zed/zed_node/imu/data'
            }],
            remappings=[
                ('/imu/data_raw', '/camera/imu'),
                ('/imu/data', '/rtabmap/imu')
            ]
        ),
        # Traversability Mapping Node
        Node(
            package='ros2_traversability_mapping',
            executable='steepness_mapping_node',
            name='steepness_mapping_node',
            output='screen',
            parameters=[zed_config_common, zed_config_specific]
        )
    ])
