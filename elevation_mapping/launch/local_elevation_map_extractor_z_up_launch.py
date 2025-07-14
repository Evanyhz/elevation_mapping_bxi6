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

    # 🚀 局部高程图提取器节点参数 - Z轴朝上版本，优化解决旋转空洞问题
    local_extractor_params = {
        'global_map_topic': '/elevation_map',
        'local_map_topic': '/local_elevation_map_z_up',
        'robot_frame': 'aft_mapped_upright',  # 使用朝上的坐标系
        'map_frame': 'camera_init',           # 更改为camera_init以匹配全局地图
        
        # 📏 扩大地图尺寸解决旋转边界问题 - 关键改进！
        'local_map_size_x': 1.6,  # 从1.6米增加到2.4米 - 为旋转预留更多空间
        'local_map_size_y': 1.0,  # 从1.0米增加到1.8米 - 确保45度旋转时不丢失数据
        
        # ⏱️ 时序优化 - 降低频率确保TF同步稳定
        'update_rate': 50.0,       # 从10Hz降低到6Hz - 给TF变换更多时间
        'tf_timeout': 0.1,        # 从1秒增加到4秒 - 大幅增加TF超时避免丢帧
        
        # 🔄 旋转配置 - 保持启用但更稳定
        'use_rotated_robot_frame': True,   # 保持启用旋转功能
        'invert_height': False,            # Z轴朝上版本不需要反转高度
        
        # 🎯 新增重采样参数 - 帮助填补旋转空洞
        'local_map_resolution': 0.0,       # 使用全局地图分辨率
        'enable_resampling': False,        # 对于旋转模式暂时禁用重采样
        'resampling_method': 'mean',       # 如果启用重采样使用均值方法
        
        # 🔧 NaN值填充参数 - 将缺失点替换为最低高度
        'enable_nan_filling': False         # 启用NaN填充功能（关键参数！）
    }

    # 🎨 可视化参数 - 针对更大地图优化
    visualization_params = {
        'grid_map_topic': '/local_elevation_map_z_up',
        'grid_map_visualization/point_cloud_height_layer': 'elevation',
        'grid_map_visualization/point_cloud_color_layer': 'elevation',
        'grid_map_visualization/point_cloud_use_rainbow': True,
        'grid_map_visualization/point_cloud_frame_id': 'aft_mapped_upright',  # 使用朝上坐标系显示
        'grid_map_visualization/point_cloud_alpha': 0.9,    # 略微透明以便观察
        'grid_map_visualization/point_cloud_height_max': 30.0,
        'grid_map_visualization/point_cloud_height_min': -10.0
    }

    return launch.LaunchDescription([
        # Launch参数
        debug_arg,
        
        # 🚀 局部高程图提取节点 - Z轴朝上版本，优化旋转空洞问题
        launch_ros.actions.Node(
            package='elevation_mapping',
            executable='local_elevation_map_extractor',
            name='local_elevation_map_extractor_z_up',
            output='screen',
            parameters=[local_extractor_params],
            arguments=['--ros-args', '--log-level', 'info']
        ),
        
        # 🎨 局部高程图可视化节点
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