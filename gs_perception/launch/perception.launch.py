from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    perception_dir = os.path.join(os.path.dirname(__file__), '..')

    return LaunchDescription([
        Node(
            package='gs_perception',
            executable='segformer_node',
            name='segformer_node',
            output='screen',
            parameters=[os.path.join(perception_dir, 'config', 'perception_params.yaml')],
            extra_env={
                'OPENVINO_DEVICE': 'GPU',
            }
        )
    ])
