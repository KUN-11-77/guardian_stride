# gs_bringup/launch/perception_only.launch.py
# 仅感知（调试用）

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    gs_perception_dir = get_package_share_directory('gs_perception')
    gs_fusion_dir     = get_package_share_directory('gs_fusion')

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
                'color_fps':         '30',
            }.items()
        ),

        # 传感器融合
        TimerAction(period=2.0, actions=[
            IncludeLaunchDescription(PythonLaunchDescriptionSource(
                os.path.join(gs_fusion_dir, 'launch', 'fusion.launch.py')))
        ]),

        # 视觉感知
        TimerAction(period=4.0, actions=[
            IncludeLaunchDescription(PythonLaunchDescriptionSource(
                os.path.join(gs_perception_dir, 'launch', 'perception.launch.py')))
        ]),
    ])
