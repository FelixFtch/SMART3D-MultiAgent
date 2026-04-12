"""
RobotCar Cartographer Launch File
Launches: cartographer_node + cartographer_occupancy_grid_node
Remaps:   /IMU_data -> /imu  (cartographer default topic)
Requires: /scan on laser_link, /IMU_data on base_link
TF:       base_link -> laser_link (static, must be running)
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # Path to lua config directory (inside container)
    config_dir = LaunchConfiguration('config_dir',
        default='/root/cartographer_config')
    config_basename = LaunchConfiguration('config_basename',
        default='robotcar_2d.lua')
    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period = LaunchConfiguration('publish_period', default='1.0')

    return LaunchDescription([

        DeclareLaunchArgument('config_dir',
            default_value='/root/cartographer_config'),
        DeclareLaunchArgument('config_basename',
            default_value='robotcar_2d.lua'),
        DeclareLaunchArgument('resolution', default_value='0.05'),
        DeclareLaunchArgument('publish_period', default_value='1.0'),

        # --- Cartographer SLAM node ---
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=[
                '-configuration_directory', config_dir,
                '-configuration_basename', config_basename,
            ],
            remappings=[
                # Remap IMU topic: hipnuc publishes /IMU_data,
                # cartographer expects /imu
                ('imu', '/IMU_data'),
                # scan is already /scan, no remap needed
            ],
        ),

        # --- Occupancy grid node (for RViz map display) ---
        # Delay 2s to let cartographer initialize first
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='cartographer_ros',
                    executable='cartographer_occupancy_grid_node',
                    name='occupancy_grid_node',
                    output='screen',
                    parameters=[{
                        'use_sim_time': False,
                        'resolution': resolution,
                        'publish_period_sec': publish_period,
                    }],
                ),
            ],
        ),
    ])
