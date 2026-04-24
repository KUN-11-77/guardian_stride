# gs_bringup/launch/hardware_only.launch.py
# 仅硬件层（调试用）

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    gs_fusion_dir = get_package_share_directory('gs_fusion')
    gs_safety_dir = get_package_share_directory('gs_safety')
    gs_exo_dir    = get_package_share_directory('gs_exo_control')

    return LaunchDescription([
        # RealSense 驱动
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('realsense2_camera'),
                             'launch', 'rs_launch.py')
            ),
            launch_arguments={
                'enable_gyro':       'true',
                'enable_accel':      'true',
                'unite_imu_method':  '1',
            }.items()
        ),

        # 传感器融合（IMU + ToF）
        TimerAction(period=2.0, actions=[
            IncludeLaunchDescription(PythonLaunchDescriptionSource(
                os.path.join(gs_fusion_dir, 'launch', 'fusion.launch.py')))
        ]),

        # 安全层
        TimerAction(period=3.0, actions=[
            Node(
                package='gs_safety',
                executable='safety_fsm_node',
                name='safety_fsm_node',
                parameters=[os.path.join(gs_safety_dir, 'config', 'safety_params.yaml')],
            )
        ]),

        # 外骨骼控制
        TimerAction(period=4.0, actions=[
            Node(
                package='gs_exo_control',
                executable='exo_controller_node',
                name='exo_controller_node',
                parameters=[os.path.join(gs_exo_dir, 'config', 'motor_params.yaml')],
            )
        ]),
    ])
