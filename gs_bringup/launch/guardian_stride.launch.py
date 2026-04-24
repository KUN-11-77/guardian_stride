# gs_bringup/launch/guardian_stride.launch.py
# 全系统启动入口：
# 1. RealSense → 2. VINS-Fusion → 3. gs_fusion → 4. gs_perception
# → 5. gs_planning (Nav2) → 6. gs_safety → 7. gs_exo_control → 8. gs_voice
#
# 重要：gs_safety 必须在 gs_exo_control 之前 Ready，
#        gs_planning 必须在 gs_perception 发布第一帧后才启动 Nav2

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 各包 share 目录
    gs_perception_dir = get_package_share_directory('gs_perception')
    gs_fusion_dir     = get_package_share_directory('gs_fusion')
    gs_planning_dir   = get_package_share_directory('gs_planning')
    gs_safety_dir     = get_package_share_directory('gs_safety')
    gs_exo_dir        = get_package_share_directory('gs_exo_control')
    gs_voice_dir      = get_package_share_directory('gs_voice')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock'
        ),

        # T=0s: RealSense 驱动
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('realsense2_camera'),
                             'launch', 'rs_launch.py')
            ),
            launch_arguments={
                'enable_gyro':       'true',
                'enable_accel':      'true',
                'unite_imu_method':  '1',
                'depth_fps':         '30',
                'color_fps':         '30',
            }.items()
        ),

        # T=2s: 传感器融合（IMU + ToF）
        TimerAction(period=2.0, actions=[
            IncludeLaunchDescription(PythonLaunchDescriptionSource(
                os.path.join(gs_fusion_dir, 'launch', 'fusion.launch.py')))
        ]),

        # T=4s: 视觉感知（等 RealSense 稳定）
        TimerAction(period=4.0, actions=[
            IncludeLaunchDescription(PythonLaunchDescriptionSource(
                os.path.join(gs_perception_dir, 'launch', 'perception.launch.py')))
        ]),

        # T=5s: 安全层（必须在控制层之前就绪）
        TimerAction(period=5.0, actions=[
            Node(
                package='gs_safety',
                executable='safety_fsm_node',
                name='safety_fsm_node',
                parameters=[os.path.join(gs_safety_dir, 'config', 'safety_params.yaml')],
                additional_env={'GOMP_SPINCOUNT': '0'},
            )
        ]),

        # T=6s: 外骨骼控制（P-Core 绑定）
        TimerAction(period=6.0, actions=[
            Node(
                package='gs_exo_control',
                executable='exo_controller_node',
                name='exo_controller_node',
                parameters=[os.path.join(gs_exo_dir, 'config', 'motor_params.yaml')],
            )
        ]),

        # T=7s: 导航规划（Nav2 + 引导力矩）
        TimerAction(period=7.0, actions=[
            IncludeLaunchDescription(PythonLaunchDescriptionSource(
                os.path.join(gs_planning_dir, 'launch', 'planning.launch.py')))
        ]),

        # T=8s: 语音交互（最后启动，不阻塞安全链路）
        TimerAction(period=8.0, actions=[
            IncludeLaunchDescription(PythonLaunchDescriptionSource(
                os.path.join(gs_voice_dir, 'launch', 'voice.launch.py')))
        ]),
    ])
