# -*- coding: utf-8 -*-
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    orbbec_camera_pkg    = get_package_share_directory('orbbec_camera')
    m3pro_navigation_pkg = get_package_share_directory('M3Pro_navigation')
    slam_mapping_pkg     = get_package_share_directory('slam_mapping')
    my_pkg               = get_package_share_directory('m3pro_auto_mapper')
    rviz_cfg = os.path.join(my_pkg, 'rviz', 'explore.rviz')
    home_map_prefix = os.path.expanduser('~/yahboom_map')

    declare = [
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('map_topic', default_value='/map'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('depth_topic', default_value='/camera/depth/image_raw'),

        # speed & plan
        DeclareLaunchArgument('linear_speed', default_value='0.14'),   # ↓ 从 0.22
        DeclareLaunchArgument('angular_speed', default_value='0.45'),
        DeclareLaunchArgument('rotate_speed',  default_value='0.30'),
        DeclareLaunchArgument('lookahead_m',   default_value='0.28'),   # ↓ 从 0.35
        DeclareLaunchArgument('replan_interval_sec', default_value='1.5'),

        # map thresholds & geometry
        DeclareLaunchArgument('free_thresh', default_value='25'),
        DeclareLaunchArgument('occ_thresh',  default_value='65'),
        DeclareLaunchArgument('robot_radius_m',   default_value='0.11'), # ↓ 从 0.17
        DeclareLaunchArgument('inflate_margin_m', default_value='0.02'), # ↓ 从 0.05
        DeclareLaunchArgument('gap_fill_enable', default_value='true'),
        DeclareLaunchArgument('gap_fill_max_m',  default_value='0.30'),

        # depth -> hard wall
        DeclareLaunchArgument('depth_roi_px', default_value='12'),      # ↑ 从 10
        DeclareLaunchArgument('depth_stop_m', default_value='0.33'),    # 从 0.34（处于建议带内）
        DeclareLaunchArgument('depth_scale_m_per_unit', default_value='0.001'),
        DeclareLaunchArgument('hard_connect_enable', default_value='true'),
        DeclareLaunchArgument('hard_connect_m', default_value='0.30'),
        DeclareLaunchArgument('hard_disk_scale', default_value='0.55'),
        DeclareLaunchArgument('hard_thin_enable', default_value='true'),
        DeclareLaunchArgument('hard_thin_radius_m', default_value='0.12'),

        # virtual TTL
        DeclareLaunchArgument('virtual_block_ttl_sec',  default_value='2.0'),
        DeclareLaunchArgument('virtual_block_radius_m', default_value='0.30'),

        # planning-only closing
        DeclareLaunchArgument('plan_close_enable', default_value='true'),
        DeclareLaunchArgument('plan_close_gap_m',  default_value='0.10'),

        # decisions
        DeclareLaunchArgument('branch_bias',        default_value='right'),
        DeclareLaunchArgument('branch_bias_weight', default_value='0.45'),
        DeclareLaunchArgument('commit_time_sec',    default_value='10.0'),
        DeclareLaunchArgument('taboo_radius_m',     default_value='0.80'),
        DeclareLaunchArgument('taboo_ttl_sec',      default_value='25.0'),

        # corridor fallback
        DeclareLaunchArgument('corridor_v',           default_value='0.14'), # ↓ 从 0.22
        DeclareLaunchArgument('corridor_k_ang',       default_value='0.9'),
        DeclareLaunchArgument('corridor_min_forward', default_value='0.35'),

        # A* clearance
        DeclareLaunchArgument('width_cost_k',      default_value='0.6'),
        DeclareLaunchArgument('min_clearance_m',   default_value='0.16'), # ↑ 从 0.14
        DeclareLaunchArgument('frontier_min_dt_m', default_value='0.18'),

        # finish criteria
        DeclareLaunchArgument('finish_stall_sec',    default_value='20.0'),  # ↓ 从 40.0
        DeclareLaunchArgument('finish_min_delta',    default_value='0.003'), # ↓ 从 0.01
        DeclareLaunchArgument('finish_min_coverage', default_value='0.80'),  # ↑ 从 0.60

        # escape
        DeclareLaunchArgument('reverse_time_sec',   default_value='1.2'),
        DeclareLaunchArgument('reverse_speed',      default_value='0.22'),
        DeclareLaunchArgument('reverse_turn_speed', default_value='0.40'),
        DeclareLaunchArgument('escape_turn_rad',    default_value=str(80/180*3.1415926)),

        # map merged
        DeclareLaunchArgument('publish_map_merged', default_value='true'),

        # save
        DeclareLaunchArgument('finish_save_pbstream', default_value='false'),
        DeclareLaunchArgument('pbstream_path',        default_value=f'{home_map_prefix}.pbstream'),
        DeclareLaunchArgument('finish_save_pgm',      default_value='true'),
        DeclareLaunchArgument('pgm_map_path',         default_value=home_map_prefix),
    ]

    explorer = ExecuteProcess(
        cmd=[
            FindExecutable(name='auto_explore_mapper'),
            '--ros-args',
            '-p', ['base_frame:=',  LaunchConfiguration('base_frame')],
            '-p', ['map_topic:=',   LaunchConfiguration('map_topic')],
            '-p', ['scan_topic:=',  LaunchConfiguration('scan_topic')],
            '-p', ['depth_topic:=', LaunchConfiguration('depth_topic')],

            '-p', ['linear_speed:=',        LaunchConfiguration('linear_speed')],
            '-p', ['angular_speed:=',       LaunchConfiguration('angular_speed')],
            '-p', ['rotate_speed:=',        LaunchConfiguration('rotate_speed')],
            '-p', ['lookahead_m:=',         LaunchConfiguration('lookahead_m')],
            '-p', ['replan_interval_sec:=', LaunchConfiguration('replan_interval_sec')],

            '-p', ['free_thresh:=', LaunchConfiguration('free_thresh')],
            '-p', ['occ_thresh:=',  LaunchConfiguration('occ_thresh')],
            '-p', ['robot_radius_m:=',   LaunchConfiguration('robot_radius_m')],
            '-p', ['inflate_margin_m:=', LaunchConfiguration('inflate_margin_m')],
            '-p', ['gap_fill_enable:=',  LaunchConfiguration('gap_fill_enable')],
            '-p', ['gap_fill_max_m:=',   LaunchConfiguration('gap_fill_max_m')],

            '-p', ['depth_roi_px:=',          LaunchConfiguration('depth_roi_px')],
            '-p', ['depth_stop_m:=',          LaunchConfiguration('depth_stop_m')],
            '-p', ['depth_scale_m_per_unit:=',LaunchConfiguration('depth_scale_m_per_unit')],
            '-p', ['hard_connect_enable:=',   LaunchConfiguration('hard_connect_enable')],
            '-p', ['hard_connect_m:=',        LaunchConfiguration('hard_connect_m')],
            '-p', ['hard_disk_scale:=',       LaunchConfiguration('hard_disk_scale')],
            '-p', ['hard_thin_enable:=',      LaunchConfiguration('hard_thin_enable')],
            '-p', ['hard_thin_radius_m:=',    LaunchConfiguration('hard_thin_radius_m')],

            '-p', ['virtual_block_ttl_sec:=',  LaunchConfiguration('virtual_block_ttl_sec')],
            '-p', ['virtual_block_radius_m:=', LaunchConfiguration('virtual_block_radius_m')],

            '-p', ['plan_close_enable:=', LaunchConfiguration('plan_close_enable')],
            '-p', ['plan_close_gap_m:=',  LaunchConfiguration('plan_close_gap_m')],

            '-p', ['branch_bias:=',        LaunchConfiguration('branch_bias')],
            '-p', ['branch_bias_weight:=', LaunchConfiguration('branch_bias_weight')],
            '-p', ['commit_time_sec:=',    LaunchConfiguration('commit_time_sec')],
            '-p', ['taboo_radius_m:=',     LaunchConfiguration('taboo_radius_m')],
            '-p', ['taboo_ttl_sec:=',      LaunchConfiguration('taboo_ttl_sec')],

            '-p', ['corridor_v:=',           LaunchConfiguration('corridor_v')],
            '-p', ['corridor_k_ang:=',       LaunchConfiguration('corridor_k_ang')],
            '-p', ['corridor_min_forward:=', LaunchConfiguration('corridor_min_forward')],

            '-p', ['width_cost_k:=',      LaunchConfiguration('width_cost_k')],
            '-p', ['min_clearance_m:=',   LaunchConfiguration('min_clearance_m')],
            '-p', ['frontier_min_dt_m:=', LaunchConfiguration('frontier_min_dt_m')],

            '-p', ['finish_stall_sec:=',    LaunchConfiguration('finish_stall_sec')],
            '-p', ['finish_min_delta:=',    LaunchConfiguration('finish_min_delta')],
            '-p', ['finish_min_coverage:=', LaunchConfiguration('finish_min_coverage')],

            '-p', ['reverse_time_sec:=',   LaunchConfiguration('reverse_time_sec')],
            '-p', ['reverse_speed:=',      LaunchConfiguration('reverse_speed')],
            '-p', ['reverse_turn_speed:=', LaunchConfiguration('reverse_turn_speed')],
            '-p', ['escape_turn_rad:=',    LaunchConfiguration('escape_turn_rad')],

            '-p', ['publish_map_merged:=', LaunchConfiguration('publish_map_merged')],

            '-p', ['finish_save_pbstream:=', LaunchConfiguration('finish_save_pbstream')],
            '-p', ['pbstream_path:=',        LaunchConfiguration('pbstream_path')],
            '-p', ['finish_save_pgm:=',      LaunchConfiguration('finish_save_pgm')],
            '-p', ['pgm_map_path:=',         LaunchConfiguration('pgm_map_path')],
        ],
        output='screen'
    )

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', rviz_cfg], condition=IfCondition(LaunchConfiguration('use_rviz')),
        output='screen'
    )

    return LaunchDescription(declare + [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(orbbec_camera_pkg, 'launch', 'dabai_dcw2.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(m3pro_navigation_pkg, 'launch', 'base_bringup.launch.py'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(slam_mapping_pkg, 'launch', 'cartographer.launch.py'))),
        rviz,
        explorer
    ])
