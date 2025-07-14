#!/usr/bin/env python3
"""
简化的机器人高程图发布器 - 不使用scipy，用简单插值
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from threading import Lock
import numpy as np
from grid_map_msgs.msg import GridMap
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout

class BxiExample(Node):
    def __init__(self):
        super().__init__('bxi_example_py')
        
        # 创建QoS配置
        qos = QoSProfile(depth=1)
        qos.reliability = qos_profile_sensor_data.reliability
        qos.durability = qos_profile_sensor_data.durability
        
        # 订阅高程图话题
        self.height_map_sub = self.create_subscription(
            GridMap, 
            '/local_elevation_map_z_up',  # 高程图话题
            self.height_map_callback, 
            qos)
            
        # 创建发布器
        self.height_map_pub = self.create_publisher(
            Float32MultiArray,
            '/robot_height_map',  # 发布转换后的高程图
            qos)
            
        self.get_logger().info("已订阅高程图话题: /elevation_map")
        self.get_logger().info("已创建发布器: /robot_height_map")
        
        # 存储地图信息
        self.map_info = None
        self.latest_elevation_map = None
        
        self.lock_in = Lock()
        self.height_map = -0.228 + np.zeros((18,9))  # 默认平地高程图

    def convert_to_robot_height_map(self, elevation_data, robot_base_height=1.1):
        """将grid map转换为机器人控制需要的高程图格式
        Args:
            elevation_data: 原始高程数据
            robot_base_height: 机器人躯干高度，默认1.1m
        Returns:
            robot_height_map: shape=(18,9)的numpy数组，相对躯干的高度
        """
        try:
            # 创建插值用的坐标网格
            x_robot = np.linspace(-0.25, 0.6, 18)  # 机器人需要的x坐标点
            y_robot = np.linspace(-0.2, 0.2, 9)    # 机器人需要的y坐标点
            
            # 获取grid map的尺寸和分辨率
            rows, cols = elevation_data.shape
            resolution = self.map_info['resolution']
            center_x = self.map_info['center_x']
            center_y = self.map_info['center_y']
            length_x = self.map_info['length_x']
            length_y = self.map_info['length_y']
            
            # 对每个机器人需要的点进行最近邻插值
            robot_height_map = -0.228 + np.zeros((18, 9))  # 初始化时就降低0.228m
            valid_points = 0
            
            for i in range(18):
                for j in range(9):
                    x = x_robot[i]
                    y = y_robot[j]
                    
                    # 找到最近的grid map格子
                    # 参考grid_map库的C++源码进行坐标变换
                    # position.x = map_center.x + length.x/2 - resolution * (col_index + 0.5)
                    # col_index = (map_center.x + length.x/2 - position.x) / resolution - 0.5
                    x_idx = int((center_x + length_x / 2.0 - x) / resolution - 0.5)
                    y_idx = int((center_y + length_y / 2.0 - y) / resolution - 0.5)
                    
                    # 确保索引在有效范围内
                    x_idx = np.clip(x_idx, 0, cols-1)
                    y_idx = np.clip(y_idx, 0, rows-1)
                    
                    # 获取高度值
                    height = elevation_data[y_idx, x_idx]
                    
                    # 处理无效值
                    if np.isnan(height) or height < -3.0:
                        height = 0.0
                    else:
                        valid_points += 1
                    
                    # 计算相对躯干的高度
                    robot_height_map[i,j] = robot_base_height - height - 0.228  # 在当前高度基础上降低0.228m
            
            # 打印调试信息
            if valid_points > 0:
                self.get_logger().info(f"有效高程点: {valid_points}/162")
                
            return robot_height_map
            
        except Exception as e:
            self.get_logger().error(f"高程图转换失败: {str(e)}")
            return -0.228 + np.zeros((18,9))  # 出错时返回降低后的平地

    def height_map_callback(self, msg):
        """处理高程图数据"""
        try:
            # 找到elevation层的索引
            elevation_idx = msg.layers.index("elevation")
            
            # 转换高程数据为numpy数组
            elevation_data = msg.data[elevation_idx]
            data_array = np.array(elevation_data.data, dtype=np.float32)
            
            # 使用实际的数据尺寸
            actual_cols = elevation_data.layout.dim[0].size  # 第一维是列
            actual_rows = elevation_data.layout.dim[1].size  # 第二维是行
            
            # 保存地图信息
            self.map_info = {
                'resolution': msg.info.resolution,
                'length_x': msg.info.length_x,
                'length_y': msg.info.length_y,
                'center_x': msg.info.pose.position.x,
                'center_y': msg.info.pose.position.y,
                'rows': actual_rows,
                'cols': actual_cols
            }
            
            # 重塑数组为正确的形状
            elevation_map = data_array.reshape((actual_rows, actual_cols))
            self.latest_elevation_map = elevation_map
            
            # 转换为机器人需要的格式
            with self.lock_in:
                self.height_map = self.convert_to_robot_height_map(elevation_map)
                
            # 发布转换后的高程图
            height_map_msg = Float32MultiArray()
            height_map_msg.layout.dim = [
                MultiArrayDimension(label="rows", size=18, stride=18*9),
                MultiArrayDimension(label="cols", size=9, stride=9)
            ]
            height_map_msg.data = self.height_map.flatten().tolist()
            self.height_map_pub.publish(height_map_msg)
                
            # 每秒打印一次调试信息
            if self.get_clock().now().nanoseconds % 1000000000 < 50000000:
                valid_data = elevation_map[~np.isnan(elevation_map)]
                valid_data = valid_data[valid_data > -1.0]
                if len(valid_data) > 0:
                    self.get_logger().info(
                        f"高程图: {actual_rows}×{actual_cols} → 18×9, " +
                        f"高程范围: [{np.min(valid_data):.3f}, {np.max(valid_data):.3f}]m"
                    )
                
        except Exception as e:
            self.get_logger().error(f"处理高程图数据失败: {str(e)}")
            # 出错时使用默认的平地高程图
            with self.lock_in:
                self.height_map = -0.228 + np.zeros((18,9))

def main(args=None):
    rclpy.init(args=args)
    node = BxiExample()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        node.get_logger().info("高程图转换器已启动")
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("正在关闭高程图转换器...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 