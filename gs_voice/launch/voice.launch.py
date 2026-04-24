# gs_voice/launch/voice.launch.py
# 语音交互模块启动文件

from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    voice_params_file = os.path.join(
        os.path.dirname(__file__), '..', 'config', 'voice_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='gs_voice',
            executable='voice_node',
            name='voice_node',
            output='screen',
            parameters=[voice_params_file],
        ),
    ])
