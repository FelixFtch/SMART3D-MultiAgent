# -*- coding: utf-8 -*-
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers.on_process_exit import OnProcessExit
from launch.events import Shutdown

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

    declare = [
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('map_topic', default_value='/map'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('depth_topic', default_value='/camera/depth/image_raw'),

        # motion & replanning
        DeclareLaunchArgument('linear_speed', default_value='0.14'),
        DeclareLaunchArgument('angular_speed', default_value='0.45'),
        DeclareLaunchArgument('rotate_speed',  default_value='0.30'),
        DeclareLaunchArgument('lookahead_m',   default_value='0.28'),
        DeclareLaunchArgument('long_lookahead_m', default_value='1.10'),
        DeclareLaunchArgument('replan_interval_sec', default_value='1.5'),
        DeclareLaunchArgument('commit_time_sec', default_value='10.0'),

        # map thresholds & geometry
        DeclareLaunchArgument('free_thresh', default_value='25'),
        DeclareLaunchArgument('occ_thresh',  default_value='65'),
        DeclareLaunchArgument('robot_radius_m',   default_value='0.11'),
        DeclareLaunchArgument('inflate_margin_m', default_value='0.02'),
        DeclareLaunchArgument('gap_fill_enable', default_value='true'),
        DeclareLaunchArgument('gap_fill_max_m',  default_value='0.30'),

        # depth -> hard wall
        DeclareLaunchArgument('depth_roi_px', default_value='12'),
        DeclareLaunchArgument('depth_stop_m', default_value='0.33'),
        DeclareLaunchArgument('depth_scale_m_per_unit', default_value='0.001'),
        DeclareLaunchArgument('hard_connect_enable', default_value='true'),
        DeclareLaunchArgument('hard_connect_m', default_value='0.30'),
        DeclareLaunchArgument('hard_disk_scale', default_value='0.55'),
        DeclareLaunchArgument('hard_thin_enable', default_value='true'),
        DeclareLaunchArgument('hard_thin_radius_m', default_value='0.12'),
        DeclareLaunchArgument('depth_clear_m', default_value='0.85'),
        DeclareLaunchArgument('depth_clear_frames', default_value='3'),

        # virtual TTL
        DeclareLaunchArgument('virtual_block_ttl_sec',  default_value='2.0'),
        DeclareLaunchArgument('virtual_block_radius_m', default_value='0.30'),

        # planning-only closing
        DeclareLaunchArgument('plan_close_enable', default_value='true'),
        DeclareLaunchArgument('plan_close_gap_m',  default_value='0.10'),

        # decisions
        DeclareLaunchArgument('branch_bias',        default_value='right'),
        DeclareLaunchArgument('branch_bias_weight', default_value='0.45'),
        DeclareLaunchArgument('forward_penalty_k',  default_value='1.2'),
        DeclareLaunchArgument('taboo_radius_m',     default_value='0.80'),
        DeclareLaunchArgument('taboo_ttl_sec',      default_value='25.0'),

        # corridor fallback
        DeclareLaunchArgument('corridor_v',           default_value='0.14'),
        DeclareLaunchArgument('corridor_k_ang',       default_value='0.9'),
        DeclareLaunchArgument('corridor_min_forward', default_value='0.35'),

        # A* clearance
        DeclareLaunchArgument('width_cost_k',      default_value='0.6'),
        DeclareLaunchArgument('min_clearance_m',   default_value='0.16'),
        DeclareLaunchArgument('frontier_min_dt_m', default_value='0.18'),

        # 路径有效性检测
        DeclareLaunchArgument('replan_on_block_enable', default_value='true'),
        DeclareLaunchArgument('block_check_ahead_m',    default_value='0.80'),
        DeclareLaunchArgument('block_detect_min_frames',default_value='1'),

        # --- NEW: Global planning & clustering ---
        DeclareLaunchArgument('use_global_tour', default_value='true'),
        DeclareLaunchArgument('cluster_method',  default_value='dbscan'),
        DeclareLaunchArgument('dbscan_eps_m',    default_value='0.60'),
        DeclareLaunchArgument('dbscan_min_samples', default_value='12'),
        DeclareLaunchArgument('cluster_downsample_stride', default_value='2'),
        DeclareLaunchArgument('max_cluster_count', default_value='12'),
        DeclareLaunchArgument('global_graph_downsample_factor', default_value='2'),
        DeclareLaunchArgument('global_plan_time_budget_sec', default_value='0.7'),
        DeclareLaunchArgument('global_replan_min_interval_sec', default_value='6.0'),
        DeclareLaunchArgument('global_replan_known_delta_ratio', default_value='0.05'),
        DeclareLaunchArgument('global_tour_strategy', default_value='ratio'),
        DeclareLaunchArgument('info_gain_enable', default_value='true'),
        DeclareLaunchArgument('info_gain_radius_m', default_value='1.6'),
        DeclareLaunchArgument('info_gain_beta', default_value='1.0'),
        DeclareLaunchArgument('arrive_thresh_m', default_value='0.40'),
        DeclareLaunchArgument('use_cluster_markers', default_value='false'),

        # bottle stop (HAVE_BOTTLE / NO_BOTTLE over UDP)
        DeclareLaunchArgument('bottle_enable', default_value='false'),
        DeclareLaunchArgument('bottle_listen_port', default_value='5005'),
        DeclareLaunchArgument('bottle_resume_delay_sec', default_value='3.0'),

        # finish criteria（交给 finisher）
        DeclareLaunchArgument('coverage_goal', default_value='0.80'),
        DeclareLaunchArgument('area_goal_m2',  default_value='100.0'),
        DeclareLaunchArgument('enclose_goal',  default_value='0.90'),
        DeclareLaunchArgument('enclose_tol_m', default_value='0.35'),
        DeclareLaunchArgument('finish_stall_sec', default_value='20.0'),
        DeclareLaunchArgument('finish_min_delta', default_value='0.003'),
        DeclareLaunchArgument('finish_stable_sec', default_value='3.0'),
        DeclareLaunchArgument('exit_after_save', default_value='false'),
        DeclareLaunchArgument('auto_finish_enable', default_value='false'),

        # save
        DeclareLaunchArgument('finish_save_pbstream', default_value='true'),
        DeclareLaunchArgument('pbstream_path',        default_value='/home/jetson/yahboom_map.pbstream'),
        DeclareLaunchArgument('finish_save_pgm',      default_value='true'),
        DeclareLaunchArgument('pgm_map_path',         default_value='/home/jetson/yahboom_map'),

        # escape
        DeclareLaunchArgument('reverse_time_sec',   default_value='1.2'),
        DeclareLaunchArgument('reverse_speed',      default_value='0.22'),
        DeclareLaunchArgument('reverse_turn_speed', default_value='0.40'),
        DeclareLaunchArgument('escape_turn_rad',    default_value=str(80/180*3.1415926)),

        # map merged
        DeclareLaunchArgument('publish_map_merged', default_value='true'),

        # --- 关闭 mapper 自保存，避免误触 ---
        DeclareLaunchArgument('self_finish_enable', default_value='false'),
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
            '-p', ['long_lookahead_m:=',    LaunchConfiguration('long_lookahead_m')],
            '-p', ['replan_interval_sec:=', LaunchConfiguration('replan_interval_sec')],
            '-p', ['commit_time_sec:=',     LaunchConfiguration('commit_time_sec')],

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
            '-p', ['depth_clear_m:=',         LaunchConfiguration('depth_clear_m')],
            '-p', ['depth_clear_frames:=',    LaunchConfiguration('depth_clear_frames')],

            '-p', ['virtual_block_ttl_sec:=',  LaunchConfiguration('virtual_block_ttl_sec')],
            '-p', ['virtual_block_radius_m:=', LaunchConfiguration('virtual_block_radius_m')],

            '-p', ['plan_close_enable:=', LaunchConfiguration('plan_close_enable')],
            '-p', ['plan_close_gap_m:=',  LaunchConfiguration('plan_close_gap_m')],

            '-p', ['branch_bias:=',        LaunchConfiguration('branch_bias')],
            '-p', ['branch_bias_weight:=', LaunchConfiguration('branch_bias_weight')],
            '-p', ['forward_penalty_k:=',  LaunchConfiguration('forward_penalty_k')],
            '-p', ['taboo_radius_m:=',     LaunchConfiguration('taboo_radius_m')],
            '-p', ['taboo_ttl_sec:=',      LaunchConfiguration('taboo_ttl_sec')],

            # bottle
            '-p', ['bottle_enable:=',           LaunchConfiguration('bottle_enable')],
            '-p', ['bottle_listen_port:=',      LaunchConfiguration('bottle_listen_port')],
            '-p', ['bottle_resume_delay_sec:=', LaunchConfiguration('bottle_resume_delay_sec')],

            '-p', ['corridor_v:=',           LaunchConfiguration('corridor_v')],
            '-p', ['corridor_k_ang:=',       LaunchConfiguration('corridor_k_ang')],
            '-p', ['corridor_min_forward:=', LaunchConfiguration('corridor_min_forward')],

            '-p', ['width_cost_k:=',      LaunchConfiguration('width_cost_k')],
            '-p', ['min_clearance_m:=',   LaunchConfiguration('min_clearance_m')],
            '-p', ['frontier_min_dt_m:=', LaunchConfiguration('frontier_min_dt_m')],

            '-p', ['replan_on_block_enable:=', LaunchConfiguration('replan_on_block_enable')],
            '-p', ['block_check_ahead_m:=',    LaunchConfiguration('block_check_ahead_m')],
            '-p', ['block_detect_min_frames:=',LaunchConfiguration('block_detect_min_frames')],

            # NEW: 全局规划参数
            '-p', ['use_global_tour:=',  LaunchConfiguration('use_global_tour')],
            '-p', ['cluster_method:=',   LaunchConfiguration('cluster_method')],
            '-p', ['dbscan_eps_m:=',     LaunchConfiguration('dbscan_eps_m')],
            '-p', ['dbscan_min_samples:=', LaunchConfiguration('dbscan_min_samples')],
            '-p', ['cluster_downsample_stride:=', LaunchConfiguration('cluster_downsample_stride')],
            '-p', ['max_cluster_count:=', LaunchConfiguration('max_cluster_count')],
            '-p', ['global_graph_downsample_factor:=', LaunchConfiguration('global_graph_downsample_factor')],
            '-p', ['global_plan_time_budget_sec:=', LaunchConfiguration('global_plan_time_budget_sec')],
            '-p', ['global_replan_min_interval_sec:=', LaunchConfiguration('global_replan_min_interval_sec')],
            '-p', ['global_replan_known_delta_ratio:=', LaunchConfiguration('global_replan_known_delta_ratio')],
            '-p', ['global_tour_strategy:=', LaunchConfiguration('global_tour_strategy')],
            '-p', ['info_gain_enable:=',  LaunchConfiguration('info_gain_enable')],
            '-p', ['info_gain_radius_m:=',LaunchConfiguration('info_gain_radius_m')],
            '-p', ['info_gain_beta:=',    LaunchConfiguration('info_gain_beta')],
            '-p', ['arrive_thresh_m:=',   LaunchConfiguration('arrive_thresh_m')],
            '-p', ['use_cluster_markers:=', LaunchConfiguration('use_cluster_markers')],

            # finish(仅供 Mapper 兜底，现默认关闭)
            '-p', ['self_finish_enable:=',  LaunchConfiguration('self_finish_enable')],

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

    finisher = ExecuteProcess(
        cmd=[
            FindExecutable(name='auto_explore_finisher'),
            '--ros-args',
            '-p', ['base_frame:=',  LaunchConfiguration('base_frame')],
            '-p', ['map_topic:=',   LaunchConfiguration('map_topic')],
            '-p', ['scan_topic:=',  LaunchConfiguration('scan_topic')],
            '-p', ['depth_topic:=', LaunchConfiguration('depth_topic')],
            # 完成/封闭/面积 + 稳定性迟滞
            '-p', ['coverage_goal:=',  LaunchConfiguration('coverage_goal')],
            '-p', ['area_goal_m2:=',   LaunchConfiguration('area_goal_m2')],
            '-p', ['enclose_goal:=',   LaunchConfiguration('enclose_goal')],
            '-p', ['enclose_tol_m:=',  LaunchConfiguration('enclose_tol_m')],
            '-p', ['finish_stall_sec:=', LaunchConfiguration('finish_stall_sec')],
            '-p', ['finish_min_delta:=', LaunchConfiguration('finish_min_delta')],
            '-p', ['finish_stable_sec:=', LaunchConfiguration('finish_stable_sec')],
            '-p', ['exit_after_save:=', LaunchConfiguration('exit_after_save')],
            '-p', ['auto_finish_enable:=', LaunchConfiguration('auto_finish_enable')],
            # 保存
            '-p', ['finish_save_pbstream:=', LaunchConfiguration('finish_save_pbstream')],
            '-p', ['pbstream_path:=',        LaunchConfiguration('pbstream_path')],
            '-p', ['finish_save_pgm:=',      LaunchConfiguration('finish_save_pgm')],
            '-p', ['pgm_map_path:=',         LaunchConfiguration('pgm_map_path')],
            # 深度急停 + PGM 阈值
            '-p', ['depth_roi_px:=',          LaunchConfiguration('depth_roi_px')],
            '-p', ['depth_stop_m:=',          LaunchConfiguration('depth_stop_m')],
            '-p', ['depth_scale_m_per_unit:=',LaunchConfiguration('depth_scale_m_per_unit')],
            '-p', ['free_thresh:=',           LaunchConfiguration('free_thresh')],
            '-p', ['occ_thresh:=',            LaunchConfiguration('occ_thresh')],
        ],
        output='screen'
    )
    finisher_exit_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=finisher,
            on_exit=[EmitEvent(event=Shutdown())]
        )
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
        explorer,
        finisher,
        finisher_exit_handler
    ])
