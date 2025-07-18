#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from grid_map_msgs.msg import GridMap
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout
import numpy as np
import time
import argparse

class ElevationMatrixExtractor(Node):
    def __init__(self, input_topic, output_topic, target_layer, publish_rate):
        super().__init__('elevation_matrix_extractor')
        
        # 设置参数
        self.input_topic = input_topic
        self.output_topic = output_topic
        self.target_layer = target_layer
        self.publish_rate = publish_rate
        
        # 创建QoS配置，确保可靠传输
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 创建订阅者和发布者
        self.subscription = self.create_subscription(
            GridMap,
            self.input_topic,
            self.grid_map_callback,
            qos
        )
        
        self.publisher = self.create_publisher(
            Float32MultiArray,
            self.output_topic,
            qos
        )
        
        # 状态变量
        self.latest_grid_map = None
        self.last_publish_time = time.time()
        
        # 创建定时器以固定频率发布矩阵
        self.timer = self.create_timer(1.0/self.publish_rate, self.publish_matrix)
        
        self.get_logger().info(f'节点初始化完成，从{self.input_topic}提取{self.target_layer}层数据并发布到{self.output_topic}')
        self.get_logger().info(f'发布频率: {self.publish_rate}Hz')
        
    def grid_map_callback(self, msg):
        """处理接收到的GridMap消息"""
        self.latest_grid_map = msg
        self.get_logger().debug('接收到新的高程图数据')
    
    def publish_matrix(self):
        """提取并发布高程矩阵，保持原始数据顺序"""
        if self.latest_grid_map is None:
            return
            
        # 查找目标层在消息中的索引
        try:
            layer_index = self.latest_grid_map.layers.index(self.target_layer)
        except ValueError:
            self.get_logger().warn(f"在高程图中未找到{self.target_layer}层！可用层: {self.latest_grid_map.layers}")
            return
            
        # 获取数据布局信息
        data_layout = self.latest_grid_map.data[layer_index].layout
        data_array = self.latest_grid_map.data[layer_index].data
        
        # 获取矩阵尺寸
        if len(data_layout.dim) != 2:
            self.get_logger().warn(f"数据维度异常: {len(data_layout.dim)}，期望2")
            return
            
        # 在GridMap中，第一维通常是列索引，第二维是行索引
        cols = data_layout.dim[0].size
        rows = data_layout.dim[1].size
        
        try:
            # 创建numpy数组，保持原始数据的一维结构
            raw_array = np.array(data_array, dtype=np.float32)
            
            # 使用原始顺序将数据reshape为二维数组
            # GridMap中数据是按列优先(column-major)顺序存储的
            matrix = raw_array.reshape((cols, rows)).transpose()
            
            # 创建标准矩阵消息
            matrix_msg = Float32MultiArray()
            
            # 设置矩阵布局 - 按行主序设置
            matrix_msg.layout.dim.append(MultiArrayDimension(
                label='rows',
                size=rows,
                stride=rows * cols
            ))
            
            matrix_msg.layout.dim.append(MultiArrayDimension(
                label='cols',
                size=cols,
                stride=cols
            ))
            
            matrix_msg.layout.data_offset = 0
            
            # 将reshape后的矩阵转回一维数组并发布
            # 使用flatten('C')确保使用行主序(C-style)展平
            matrix_msg.data = matrix.flatten('C').tolist()
            
            # 发布矩阵消息
            self.publisher.publish(matrix_msg)
            self.get_logger().debug(f'发布了高程矩阵: {rows}x{cols}')
            
        except Exception as e:
            self.get_logger().error(f'处理高程数据时出错: {e}')

def main():
    # 解析命令行参数
    parser = argparse.ArgumentParser(
        description='从GridMap中提取高程矩阵并发布为Float32MultiArray'
    )
    parser.add_argument(
        '--input-topic', 
        default='/local_elevation_map_z_up',
        help='输入高程图话题名称'
    )
    parser.add_argument(
        '--output-topic', 
        default='/extracted_elevation_matrix',
        help='输出矩阵话题名称'
    )
    parser.add_argument(
        '--target-layer', 
        default='elevation',
        help='要提取的目标层名称'
    )
    parser.add_argument(
        '--publish-rate', 
        type=float, 
        default=10.0,
        help='发布频率(Hz)'
    )
    
    args = parser.parse_args()
    
    # 初始化ROS
    rclpy.init()
    
    # 创建节点
    extractor = ElevationMatrixExtractor(
        args.input_topic,
        args.output_topic,
        args.target_layer,
        args.publish_rate
    )
    
    try:
        print(f"开始提取高程矩阵，按 Ctrl+C 停止...")
        rclpy.spin(extractor)
    except KeyboardInterrupt:
        print("用户中断，正在停止...")
    finally:
        extractor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()