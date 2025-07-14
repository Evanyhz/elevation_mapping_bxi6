import os
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # 获取包路径
    share_dir = get_package_share_directory('elevation_mapping')
    
    # 使用Fast-LIVO2配置文件
    config_file = os.path.join(share_dir, 'config', 'robots', 'fastlivo2_robot_config.yaml')
    
    # Fast-LIVO2优化配置参数 - 减少里程计抖动
    params = {
        'map_frame_id': 'map',
        'robot_base_frame_id': 'camera_init',
        'track_point_frame_id': 'camera_init',
        'robot_pose_with_covariance_topic': '/aft_mapped_to_init',
        'robot_pose_cache_size': 300,  # 增加缓存以提高稳定性
        'inputs': ['fastlivo2_cloud'],
        'fastlivo2_cloud.type': 'pointcloud',
        'fastlivo2_cloud.topic': '/cloud_registered',
        'fastlivo2_cloud.queue_size': 8,  # 适当增加队列以减少丢帧
        'fastlivo2_cloud.publish_on_update': False,
        'fastlivo2_cloud.sensor_processor.type': 'laser',
        'track_point_x': 0.0,
        'track_point_y': 0.0,
        'track_point_z': 0.0,
        'min_update_rate': 2.0,  # 提高最小更新率以保持稳定性
        'fused_map_publishing_rate': 2.0,  # 提高地图发布率以获得更平滑的可视化
        'time_tolerance': 0.5,  # 减少时间容忍度以提高同步精度
        'time_offset_for_point_cloud': 0.0,
        'length_in_x': 25.0,
        'length_in_y': 25.0,
        'position_x': 0.0,
        'position_y': 0.0,
        'resolution': 0.01,  # ✅ 1cm高精度全局地图
        'min_variance': 0.0001,
        'max_variance': 0.12,  # 稍微降低最大方差
        'mahalanobis_distance_threshold': 3.0,  # 增加阈值以减少敏感性
        'multi_height_noise': 0.001,  # 减少多高度噪声
        'sensor_processor.ignore_points_above': 6.0,
        'sensor_processor.ignore_points_below': -2.5,
        'sensor_processor.apply_voxelgrid_filter': True,  # 启用体素滤波减少噪声
        'sensor_processor.voxelgrid_filter_size': 0.05,   # 5cm体素大小
        'enable_visibility_cleanup': False
        'visibility_cleanup_rate': 1.0,  # 提高清理频率以保持地图质量
        'scanning_duration': 1.5,  # 适当减少扫描持续时间
        
        # 关键：机器人运动更新配置 - 减少里程计抖动
        'robot_motion_map_update.covariance_scale': 0.15,  # 增加协方差缩放
        'robot_motion_map_update.translation_variance_threshold': 0.01,
        'robot_motion_map_update.rotation_variance_threshold': 0.01
    }
    
    return launch.LaunchDescription([
        # 添加map到camera_init的静态变换
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_camera_init_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'camera_init'],
            output='screen'
        ),
        
        # Elevation Mapping节点 - 使用Fast-LIVO2的数据和TF变换
        launch_ros.actions.Node(
            package='elevation_mapping',
            executable='elevation_mapping',
            name='elevation_mapping',
            output='screen',
            parameters=[params],
            arguments=['--ros-args', '--log-level', 'info']
        ),
        
        # 等待一下再启动可视化
        launch.actions.TimerAction(
            period=5.0,
            actions=[
                # Grid Map可视化节点 - 优化参数
                launch_ros.actions.Node(
                    package='grid_map_visualization',
                    executable='grid_map_visualization',
                    name='elevation_map_visualization',
                    output='screen',
                    parameters=[{
                        'grid_map_topic': '/elevation_map',
                        'grid_map_visualization/point_cloud_height_layer': 'elevation',
                        'grid_map_visualization/point_cloud_color_layer': 'elevation',
                        'grid_map_visualization/point_cloud_use_rainbow': True,
                        'grid_map_visualization/point_cloud_frame_id': 'map'
                    }]
                ),
                
                # RViz2可视化 - 优化性能参数
                launch_ros.actions.Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=[
                        '-d', os.path.join(share_dir, 'rviz2', 'fastlivo2_demo.rviz'),
                        '--ros-args',
                        '--remap', '__node:=rviz2_optimized',
                        '--param', 'use_sim_time:=false'
                    ],
                    output='screen'
                )
            ]
        )
    ]) 