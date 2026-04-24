# gs_exo_control/launch/exo_control.launch.py
# 外骨骼控制模块启动文件

from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    motor_params_file = os.path.join(
        os.path.dirname(__file__), '..', 'config', 'motor_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='gs_exo_control',
            executable='exo_controller_node',
            name='exo_controller_node',
            output='screen',
            parameters=[motor_params_file],
            arguments=['--ros-args', '--log-level', 'info'],
        ),
    ])
