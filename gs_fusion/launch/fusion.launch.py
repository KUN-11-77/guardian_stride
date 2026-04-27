from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    fusion_dir = get_package_share_directory('gs_fusion')
    return LaunchDescription([
        Node(package='gs_fusion', executable='imu_node', name='imu_node',
             parameters=[os.path.join(fusion_dir, 'config', 'imu_params.yaml')]),
        Node(package='gs_fusion', executable='tof_node', name='tof_node'),
        Node(package='gs_fusion', executable='occupancy_bridge', name='occupancy_bridge'),
    ])
