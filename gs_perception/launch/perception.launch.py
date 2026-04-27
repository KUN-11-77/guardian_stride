from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    percep_dir = get_package_share_directory('gs_perception')
    return LaunchDescription([
        Node(package='gs_perception', executable='segformer_node', name='segformer_node',
             parameters=[os.path.join(percep_dir, 'config', 'perception_params.yaml')]),
        Node(package='gs_perception', executable='costmap_bridge', name='costmap_bridge'),
    ])
