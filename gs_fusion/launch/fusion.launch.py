from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    ekf_params_file = os.path.join(
        os.path.dirname(__file__), '../config/ekf_params.yaml'
    )

    return LaunchDescription([
        # IMU 节点
        Node(
            package='gs_fusion',
            executable='imu_node',
            name='imu_node',
            output='screen',
            parameters=[{
                'i2c_bus': 1,
                'i2c_address': 0x28,
                'frame_id': 'imu_link',
                'sampling_rate_hz': 1000,
            }]
        ),

        # ToF 节点
        Node(
            package='gs_fusion',
            executable='tof_node',
            name='tof_node',
            output='screen',
            parameters=[{
                'i2c_bus': 0,
                'frame_id': 'tof_link',
                'sampling_rate_hz': 50,
            }]
        ),

        # EKF 融合（使用 robot_localization ekf_node）
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_params_file],
            remappings=[
                ('odometry/filtered', '/state'),
            ]
        ),
    ])
