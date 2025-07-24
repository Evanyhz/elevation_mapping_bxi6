import os
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # 获取包路径
    share_dir = get_package_share_directory('elevation_mapping')
    
    # 使用Fast-LIVO2配置文件，但创建Z轴朝上的版本
    config_file = os.path.join(share_dir, 'config', 'robots')
    
    # Fast-LIVO2 Z轴朝上配置参数
    params = {
        'map_frame_id': 'map',
        'robot_base_frame_id': 'aft_mapped_upright',  # 使用新的朝上坐标系
        'track_point_frame_id': 'aft_mapped_upright', # 使用新的朝上坐标系
        'robot_pose_with_covariance_topic': '/aft_mapped_to_init',
        'robot_pose_cache_size': 300,
        'inputs': ['fastlivo2_cloud'],
        'fastlivo2_cloud.type': 'pointcloud',
        'fastlivo2_cloud.topic': '/cloud_registered',
        'fastlivo2_cloud.queue_size': 1,
        'fastlivo2_cloud.publish_on_update': False,
        'fastlivo2_cloud.sensor_processor.type': 'laser',
        'track_point_x': 0.75,    #机器人距矩形中心 X 方向的偏移
        'track_point_y': 0.0,
        'track_point_z': 0.0,
        'min_update_rate': 50.0,
        'fused_map_publishing_rate': 10.0,
        'time_tolerance': 1.0,
        'time_offset_for_point_cloud': 0.0,
        'length_in_x': 4.0,   # ✅ 长
        'length_in_y': 4.0,   # ✅ 宽
        'resolution': 0.0333,   # ✅ 分辨率
        'position_x': 0.0,
        'position_y': 0.0,
        'min_variance': 0.005,
        'max_variance': 0.01,
        'mahalanobis_distance_threshold': 2.5,
        'multi_height_noise': 0.0000009,
        'sensor_processor.ignore_points_above': 10.0,
        'sensor_processor.ignore_points_below': -10.5,
        'sensor_processor.apply_voxelgrid_filter': True,
        'sensor_processor.voxelgrid_filter_size': 0.05,
        'enable_visibility_cleanup': True,
        'visibility_cleanup_rate': 1.0,
        'scanning_duration': 0.0,
        
        # 机器人运动更新配置
        'robot_motion_map_update.covariance_scale': 0.15,
        'robot_motion_map_update.translation_variance_threshold': 0.01,
        'robot_motion_map_update.rotation_variance_threshold': 0.01
    }
    
    return launch.LaunchDescription([
        # 步骤1: map到camera_init的静态变换（保持原有变换）
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_camera_init_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'camera_init'],
            output='screen'
        ),
        
        # 步骤2: 先绕X轴旋转180度翻转Z轴     #-0.011, -0.02329, 0.04412
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='aft_mapped_to_z_up_tf',
            arguments=[
                '-0.011', '-0.02329', '0.04412',           # 位移: x, y, z
                '0', '0', '3.14159',     # 旋转: yaw, pitch, roll (roll=180度绕X轴)
                'aft_mapped', 'aft_mapped_z_up'
            ],
            output='screen'
        ),
        
        # 步骤3: 在Z轴朝上基础上绕Z轴逆时针旋转90度
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='z_up_to_upright_tf',
            arguments=[
                '0', '0', '0',           # 位移: x, y, z
                '-1.5708', '0', '0',      # 旋转: yaw=90度 (绕Z轴逆时针)
                'aft_mapped_z_up', 'aft_mapped_upright'
            ],
            output='screen'
        ),
        
        # 步骤3: Elevation Mapping节点 - 使用新的朝上坐标系
        launch_ros.actions.Node(
            package='elevation_mapping',
            executable='elevation_mapping',
            name='elevation_mapping_z_up',
            output='screen',
            parameters=[params],
            arguments=['--ros-args', '--log-level', 'info']
        ),
        
        # 等待一下再启动可视化
        launch.actions.TimerAction(
            period=5.0,
            actions=[
                # Grid Map可视化节点
                launch_ros.actions.Node(
                    package='grid_map_visualization',
                    executable='grid_map_visualization',
                    name='elevation_map_visualization_z_up',
                    output='screen',
                    parameters=[{
                        'grid_map_topic': '/elevation_map',
                        'grid_map_visualization/point_cloud_height_layer': 'elevation',
                        'grid_map_visualization/point_cloud_color_layer': 'elevation',
                        'grid_map_visualization/point_cloud_use_rainbow': True,
                        'grid_map_visualization/point_cloud_frame_id': 'map'
                    }]
                ),
                
                # RViz2可视化 - 使用Z轴朝上专用配置
                launch_ros.actions.Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2_z_up',
                    arguments=[
                        '-d', os.path.join(share_dir, 'rviz2', 'fastlivo2_z_up_demo.rviz'),
                        '--ros-args',
                        '--remap', '__node:=rviz2_z_up',
                        '--param', 'use_sim_time:=false'
                    ],
                    output='screen'
                )
            ]
        )
    ]) 
