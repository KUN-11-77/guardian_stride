# gs_planning/launch/planning.launch.py
# 启动 Nav2 导航栈 + guidance_torque_node + intent_to_goal

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    nav2_params_file = os.path.join(
        os.path.dirname(__file__), '..', 'config', 'nav2_params.yaml'
    )
    guidance_params_file = os.path.join(
        os.path.dirname(__file__), '..', 'config', 'guidance_params.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock'
        ),

        # Nav2 生命周期管理器
        TimerAction(
            period=1.0,
            actions=[
                Node(
                    package='nav2_controller',
                    executable='controller_server',
                    name='controller_server',
                    output='screen',
                    parameters=[nav2_params_file],
                    arguments=['--ros-args', '--log-level', 'info'],
                ),
            ]
        ),

        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='nav2_planner',
                    executable='planner_server',
                    name='planner_server',
                    output='screen',
                    parameters=[nav2_params_file],
                    arguments=['--ros-args', '--log-level', 'info'],
                ),
            ]
        ),

        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='nav2_behaviors',
                    executable='behavior_server',
                    name='behavior_server',
                    output='screen',
                    parameters=[nav2_params_file],
                    arguments=['--ros-args', '--log-level', 'info'],
                ),
            ]
        ),

        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='nav2_smoother',
                    executable='smoother_server',
                    name='smoother_server',
                    output='screen',
                    parameters=[nav2_params_file],
                    arguments=['--ros-args', '--log-level', 'info'],
                ),
            ]
        ),

        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='nav2_velocity_smoother',
                    executable='velocity_smoother',
                    name='velocity_smoother',
                    output='screen',
                    parameters=[nav2_params_file],
                    arguments=['--ros-args', '--log-level', 'info'],
                ),
            ]
        ),

        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager',
                    output='screen',
                    parameters=[nav2_params_file],
                    arguments=['--ros-args', '--log-level', 'info'],
                ),
            ]
        ),

        # 引导力矩计算节点
        Node(
            package='gs_planning',
            executable='guidance_torque_node',
            name='guidance_torque_node',
            output='screen',
            parameters=[guidance_params_file],
        ),

        # Intent → Goal 桥接节点
        Node(
            package='gs_planning',
            executable='intent_to_goal',
            name='intent_to_goal',
            output='screen',
        ),
    ])
