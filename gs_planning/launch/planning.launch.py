from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    plan_dir = get_package_share_directory('gs_planning')
    return LaunchDescription([
        Node(package='gs_planning', executable='guidance_torque_node', name='guidance_torque_node',
             parameters=[os.path.join(plan_dir, 'config', 'guidance_params.yaml')]),
        Node(package='gs_planning', executable='intent_to_goal', name='intent_to_goal_node'),
    ])
