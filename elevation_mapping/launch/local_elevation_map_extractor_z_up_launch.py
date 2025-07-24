import os
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # 声明launch参数
    debug_arg = launch.actions.DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Enable debug mode with console output'
    )
    local_extractor_params = {
        'global_map_topic': '/elevation_map',
        'local_map_topic': '/local_elevation_map_z_up',
        'robot_frame': 'aft_mapped_upright',  # 使用朝上的坐标系
        'map_frame': 'camera_init',           # 更改为camera_init以匹配全局地图

        'local_map_size_x': 1.20,  # 局部高程图的长
        'local_map_size_y': 0.40,  # 全局高程图的长
        'update_rate': 500.0,      
        'tf_timeout': 0.01,       
        # 旋转
        'use_rotated_robot_frame': True,  
        'invert_height': False,            
    
        # 添加以下三个参数来设置局部高程图的分辨率
        'enable_resampling': True,         
        'local_map_resolution': 0.05,      # 5cm分辨率
        'resampling_method': 'mean'      
    }
    # 可视化参数
    visualization_params = {
        'grid_map_topic': '/local_elevation_map_z_up',
        'grid_map_visualization/point_cloud_height_layer': 'elevation',
        'grid_map_visualization/point_cloud_color_layer': 'elevation',
        'grid_map_visualization/point_cloud_use_rainbow': True,
        'grid_map_visualization/point_cloud_frame_id': 'aft_mapped_upright',  # 使用朝上坐标系显示
        'grid_map_visualization/point_cloud_alpha': 1.0,    
        'grid_map_visualization/point_cloud_height_max': 30.0,
        'grid_map_visualization/point_cloud_height_min': -30.0
    }

    return launch.LaunchDescription([
        debug_arg,
        launch_ros.actions.Node(
            package='elevation_mapping',
            executable='local_elevation_map_extractor',
            name='local_elevation_map_extractor_z_up',
            output='screen',
            parameters=[local_extractor_params],
            arguments=['--ros-args', '--log-level', 'info']
        ),
        launch_ros.actions.Node(
            package='grid_map_visualization',
            executable='grid_map_visualization',
            name='local_elevation_map_visualization_z_up',
            output='screen',
            parameters=[visualization_params],
            condition=launch.conditions.IfCondition(
                launch.substitutions.PythonExpression([
                    "'", launch.substitutions.LaunchConfiguration('debug'), "' == 'true'"
                ])
            )
        )
    ])
