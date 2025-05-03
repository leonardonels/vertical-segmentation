from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('vertical-segmentation'),
        'config',
        'params.yaml'
        )
        
    return LaunchDescription([
        Node(
            package='vertical-segmentation',
            executable='pointcloud_filter_node',
            parameters=[config],
            name='pointcloud_filter_node',
            output='screen',
        ),
    ])
