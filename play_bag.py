import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py

def read_robot_height_map_from_bag(bag_file_path):
    # --- 初始化读取器，这部分和之前一样 ---
    storage_options = rosbag2_py.StorageOptions(uri=bag_file_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    # --- 获取话题和消息类型，现在是针对 Float32MultiArray ---
    topic_types = reader.get_all_topics_and_types()
    type_map = {topic_meta.name: topic_meta.type for topic_meta in topic_types}
    
    topic_name = '/extracted_elevation_matrix'
    if topic_name not in type_map:
        print(f"错误: 在bag文件中找不到话题 '{topic_name}'。")
        return
        
    # 获取正确的消息类型定义
    msg_type = get_message(type_map[topic_name])

    all_matrices = []  # 创建一个列表，用来存储所有矩阵

    # --- 核心修改：循环读取并按新格式解析 ---
    while reader.has_next():
        (topic, data, t) = reader.read_next()
        if topic == topic_name:
            msg = deserialize_message(data, msg_type)
            
            # 1. 从 layout.dim 中获取矩阵的维度
            rows = msg.layout.dim[0].size
            cols = msg.layout.dim[1].size
            
            # 2. 将一维的 data 数组，重新塑形 (reshape) 成二维矩阵
            matrix = np.array(msg.data, dtype=np.float32).reshape((rows, cols))           
            all_matrices.append(matrix)
    print(f"--- Bag 读取完成 ---")
    print(f"成功从话题 '{topic_name}' 中读取了 {len(all_matrices)} 个矩阵。")

    # --- 保存为记事本可读的 .txt 文件，这部分和之前一样 ---
    if all_matrices:
        output_txt_path = 'robot_height_map_data.txt'
        print(f"正在将所有矩阵保存到 '{output_txt_path}'...")
        
        with open(output_txt_path, 'w') as f:
            for i, matrix in enumerate(all_matrices):
                f.write(f"--- 矩阵序号 {i + 1} ---\n")
                np.savetxt(f, matrix, fmt='%.8f', delimiter=', ')
                f.write("\n\n")
                
        print(f"所有矩阵已成功保存到 '{output_txt_path}'")

if __name__ == '__main__':
    # 请在这里替换成您实际的bag文件夹名
    bag_path = 'rosbag2_2025_07_17-18_47_38'
    read_robot_height_map_from_bag(bag_path)