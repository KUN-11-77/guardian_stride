"""guardian_voice 启动文件：一次性拉起 4 个节点。

需要 RealSense 已经在跑（launch realsense2_camera 或单独启动）。
默认话题约定：
  /camera/camera/depth/image_rect_raw → depth_obstacle_node
  /voice/text_in                      → voice_asr_node → voice_assistant_node
  /tts/say                            → voice_assistant_node → tts_node
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('guardian_voice')
    params_file = os.path.join(pkg_dir, 'config', 'voice_params.yaml')

    common = {'parameters': [params_file]}

    return LaunchDescription([
        Node(
            package='guardian_voice',
            executable='voice_asr_node',
            name='voice_asr_node',
            output='screen',
            **common,
        ),
        Node(
            package='guardian_voice',
            executable='depth_obstacle_node',
            name='depth_obstacle_node',
            output='screen',
            **common,
        ),
        Node(
            package='guardian_voice',
            executable='voice_assistant_node',
            name='voice_assistant_node',
            output='screen',
            **common,
        ),
        Node(
            package='guardian_voice',
            executable='tts_node',
            name='tts_node',
            output='screen',
            **common,
        ),
    ])
