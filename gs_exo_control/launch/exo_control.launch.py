from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    exo_dir = get_package_share_directory('gs_exo_control')
    return LaunchDescription([
        Node(package='gs_exo_control', executable='exo_controller_node', name='exo_controller_node',
             parameters=[os.path.join(exo_dir, 'config', 'motor_params.yaml')]),
    ])
