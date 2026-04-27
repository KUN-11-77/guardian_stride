# gs_safety/launch/safety.launch.py
# M5 安全反射层启动

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    safety_dir = get_package_share_directory('gs_safety')

    return LaunchDescription([
        Node(
            package='gs_safety',
            executable='safety_fsm_node',
            name='safety_fsm_node',
            parameters=[os.path.join(safety_dir, 'config', 'safety_params.yaml')],
            additional_env={'GOMP_SPINCOUNT': '0'},
        ),
    ])
