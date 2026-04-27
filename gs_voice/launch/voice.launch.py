from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    voice_dir = get_package_share_directory('gs_voice')
    return LaunchDescription([
        Node(package='gs_voice', executable='voice_node', name='voice_node',
             parameters=[os.path.join(voice_dir, 'config', 'voice_params.yaml')]),
    ])
