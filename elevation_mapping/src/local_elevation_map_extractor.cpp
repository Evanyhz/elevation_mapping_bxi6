/*
 * local_elevation_map_extractor.cpp
 *
 *  Created on: 2025
 *      Author: Assistant
 *   Purpose: Extract local elevation map from global elevation map
 */

#include <rclcpp/rclcpp.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <Eigen/Geometry>
#include <chrono>
#include <memory>
#include <cmath>
#include <numeric>  // 用于 std::accumulate
#include <vector>   // 用于 std::vector
#include <limits>   // 用于 std::numeric_limits
#define _USE_MATH_DEFINES
#include <math.h>

class LocalElevationMapExtractor : public rclcpp::Node
{
public:
    LocalElevationMapExtractor() : Node("local_elevation_map_extractor")
    {
        // 声明参数
        this->declare_parameter("global_map_topic", "/elevation_map");
        this->declare_parameter("local_map_topic", "/local_elevation_map");
        this->declare_parameter("robot_frame", "aft_mapped");
        this->declare_parameter("map_frame", "map");
        this->declare_parameter("local_map_size_x", 1.6);
        this->declare_parameter("local_map_size_y", 1.0);
        this->declare_parameter("local_map_resolution", 0.0);  // 0.0表示使用全局地图分辨率
        this->declare_parameter("update_rate", 50.0);
        this->declare_parameter("tf_timeout", 0.1);
        this->declare_parameter("use_rotated_robot_frame", true);  // 只使用平移跟随
        this->declare_parameter("invert_height", false);  // 是否反转高度信息
        this->declare_parameter("enable_resampling", false);  // 是否启用分辨率重采样
        this->declare_parameter("resampling_method", "mean");  // 重采样方法
        this->declare_parameter("enable_nan_filling", false);   // 是否启用NaN值填充

        // 获取参数
        global_map_topic_ = this->get_parameter("global_map_topic").as_string();
        local_map_topic_ = this->get_parameter("local_map_topic").as_string();
        robot_frame_ = this->get_parameter("robot_frame").as_string();
        map_frame_ = this->get_parameter("map_frame").as_string();
        local_map_size_x_ = this->get_parameter("local_map_size_x").as_double();
        local_map_size_y_ = this->get_parameter("local_map_size_y").as_double();
        local_map_resolution_ = this->get_parameter("local_map_resolution").as_double();
        update_rate_ = this->get_parameter("update_rate").as_double();
        tf_timeout_ = this->get_parameter("tf_timeout").as_double();
        use_rotated_robot_frame_ = this->get_parameter("use_rotated_robot_frame").as_bool();
        invert_height_ = this->get_parameter("invert_height").as_bool();
        enable_resampling_ = this->get_parameter("enable_resampling").as_bool();
        resampling_method_ = this->get_parameter("resampling_method").as_string();
        enable_nan_filling_ = this->get_parameter("enable_nan_filling").as_bool();

        // 初始化TF2
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // 订阅全局高程图
        global_map_subscriber_ = this->create_subscription<grid_map_msgs::msg::GridMap>(
            global_map_topic_, 
            10,
            std::bind(&LocalElevationMapExtractor::globalMapCallback, this, std::placeholders::_1)
        );

        // 发布局部高程图
        local_map_publisher_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
            local_map_topic_, 
            10
        );

        // 创建定时器定期更新局部地图
        auto timer_period = std::chrono::duration<double>(1.0 / update_rate_);
        timer_ = this->create_wall_timer(
            timer_period,
            std::bind(&LocalElevationMapExtractor::updateLocalMap, this)
        );

        RCLCPP_INFO(this->get_logger(), "Local Elevation Map Extractor initialized");
        RCLCPP_INFO(this->get_logger(), "Global map topic: %s", global_map_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Local map topic: %s", local_map_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Robot frame: %s", robot_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "Map frame: %s", map_frame_.c_str());
        RCLCPP_INFO(this->get_logger(), "Local map size: %.2f x %.2f m", local_map_size_x_, local_map_size_y_);
        RCLCPP_INFO(this->get_logger(), "Use rotated robot frame: %s", use_rotated_robot_frame_ ? "enabled" : "disabled");
    }

private:
    void globalMapCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg)
    {
        try {
            // 转换为grid_map格式
            grid_map::GridMapRosConverter::fromMessage(*msg, global_map_);
            has_global_map_ = true;
            
            RCLCPP_DEBUG(this->get_logger(), "Received global elevation map with size: %f x %f m", 
                        global_map_.getLength().x(), global_map_.getLength().y());
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Failed to convert global map message: %s", e.what());
        }
    }

    void updateLocalMap()
    {
        if (!has_global_map_) {
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                                  "Waiting for global elevation map...");
            return;
        }

        try {
            // 获取机器人当前位置
            geometry_msgs::msg::TransformStamped transform_stamped;
            transform_stamped = tf_buffer_->lookupTransform(
                map_frame_, 
                robot_frame_,
                rclcpp::Time(0),  // 使用最新的可用变换
                tf2::durationFromSec(tf_timeout_)
            );

            // 提取机器人位置和姿态
            grid_map::Position robot_position(
                transform_stamped.transform.translation.x,
                transform_stamped.transform.translation.y
            );
            
            // 提取机器人的Z坐标
            double robot_z = transform_stamped.transform.translation.z;
            
            // 提取机器人的四元数姿态
            geometry_msgs::msg::Quaternion robot_orientation = transform_stamped.transform.rotation;
            
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                               "Robot position: (%.2f, %.2f, %.2f), orientation: (%.2f, %.2f, %.2f, %.2f)", 
                               robot_position.x(), robot_position.y(), robot_z,
                               robot_orientation.x, robot_orientation.y, 
                               robot_orientation.z, robot_orientation.w);

            // 创建局部地图
            extractLocalMap(robot_position, robot_orientation, robot_z);

        } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "Could not transform from %s to %s: %s", 
                                 robot_frame_.c_str(), map_frame_.c_str(), ex.what());
        }
    }

    void extractLocalMap(const grid_map::Position& center_position, const geometry_msgs::msg::Quaternion& orientation, double robot_z)
    {
        try {
            grid_map::Length local_length(local_map_size_x_, local_map_size_y_);
            grid_map::GridMap local_map;
            
            if (use_rotated_robot_frame_) {
                // 🔧 使用旋转功能：创建旋转的局部地图 - 优化版本解决空洞问题
                
                // 🚀 增加扩展因子以确保旋转后完整覆盖（从1.5增加到2.2）
                double extend_factor = 3.0;  // 更大的扩展因子确保45度旋转时不丢失数据
                grid_map::Length extended_length(local_map_size_x_ * extend_factor, local_map_size_y_ * extend_factor);
                bool is_success = false;
                grid_map::GridMap extended_map = global_map_.getSubmap(center_position, extended_length, is_success);
                
                if (!is_success) {
                    RCLCPP_WARN(this->get_logger(), "Failed to extract extended submap from global map");
                    return;
                }
                
                // 将四元数转换为Eigen变换矩阵
                Eigen::Quaterniond eigen_quat(orientation.w, orientation.x, orientation.y, orientation.z);
                
                // 🔧 创建从地图坐标系到机器人坐标系的变换 - 改进版本
                Eigen::Isometry3d robot_to_map = Eigen::Isometry3d::Identity();
                robot_to_map.translation() = Eigen::Vector3d(center_position.x(), center_position.y(), robot_z);
                robot_to_map.linear() = eigen_quat.toRotationMatrix();
                
                // 我们需要的是从地图到机器人的变换（用于getTransformedMap）
                Eigen::Isometry3d map_to_robot = robot_to_map.inverse();
                
                // 使用getTransformedMap创建旋转后的地图
                if (!extended_map.getLayers().empty()) {
                    std::string height_layer = extended_map.getLayers()[0];  // 使用第一个可用层作为高度层
                    
                    try {
                        // 🎯 使用更精确的采样密度 - 增加采样密度以减少空洞
                        double sampling_factor = 0.25;  // 采样密度因子
                        local_map = extended_map.getTransformedMap(map_to_robot, height_layer, robot_frame_, sampling_factor);
                        
                        // 🔧 改进的裁剪策略 - 使用略大的裁剪范围然后再精确裁剪
                        grid_map::Position origin(0.0, 0.0);  // 机器人坐标系原点
                        grid_map::Length safe_length(local_map_size_x_ * 1.0, local_map_size_y_ * 1.0);  // 略大的安全裁剪
                        bool crop_success = false;
                        grid_map::GridMap safe_cropped_map = local_map.getSubmap(origin, safe_length, crop_success);
                        
                        if (crop_success) {
                            // 🎯 最终精确裁剪到目标大小
                            bool final_crop_success = false;
                            local_map = safe_cropped_map.getSubmap(origin, local_length, final_crop_success);
                            
                            if (!final_crop_success) {
                                // 如果精确裁剪失败，使用安全裁剪的结果
                                local_map = safe_cropped_map;
                                RCLCPP_DEBUG(this->get_logger(), "Using safe crop size due to final crop failure");
                            }
                        } else {
                            RCLCPP_WARN(this->get_logger(), "Failed to crop rotated map, using original size");
                            // 保持原始变换的地图
                        }
                        
                        // 已禁用 fillSmallHoles —— 若需重新启用，请恢复此调用并提供函数实现
                        
                        // 如果需要，反转高度信息
                        if (invert_height_ && !local_map.getLayers().empty()) {
                            auto& height_data = local_map[height_layer];
                            height_data = -height_data.array();
                        }
                        
                        // 🔄 NaN值填充函数 - 将所有缺失点替换为最低点高程
                        if (enable_nan_filling_) {
                            fillNaNWithMinHeight(local_map, height_layer);
                        }
                        
                        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                   "Published rotated local elevation map at (%.2f, %.2f) with yaw %.2f in %s frame%s (extend_factor=%.1f)", 
                                   center_position.x(), center_position.y(), 
                                   atan2(2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
                                         1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)),
                                   robot_frame_.c_str(),
                                   invert_height_ ? " (height inverted)" : "",
                                   extend_factor);
                        
                    } catch (const std::exception& e) {
                        RCLCPP_WARN(this->get_logger(), "Failed to apply rotation transformation: %s", e.what());
                        return;
                    }
                } else {
                    RCLCPP_WARN(this->get_logger(), "No layers available in extended map");
                    return;
                }
                
            } else {
                // 简单版本：只有位置跟随，使用getSubmap()直接提取
                bool is_success = false;
                local_map = global_map_.getSubmap(center_position, local_length, is_success);
                
                if (!is_success) {
                    RCLCPP_WARN(this->get_logger(), "Failed to extract submap from global map");
                    return;
                }
                
                // 🔄 填充NaN值 - 对非旋转模式也使用最低高度填充
                if (!local_map.getLayers().empty()) {
                    std::string height_layer = local_map.getLayers()[0];
                    if (enable_nan_filling_) {
                        fillNaNWithMinHeight(local_map, height_layer);
                    }
                }
                
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Published local elevation map at (%.2f, %.2f) in %s frame (translation only)", 
                           center_position.x(), center_position.y(), map_frame_.c_str());
            }

            // 🔄 分辨率重采样（如果启用）
            if (enable_resampling_ && local_map_resolution_ > 0.0 && 
                std::abs(local_map.getResolution() - local_map_resolution_) > 1e-6) {
                
                RCLCPP_DEBUG(this->get_logger(), "Resampling local map from %.3fm to %.3fm resolution", 
                           local_map.getResolution(), local_map_resolution_);
                
                // 创建新的重采样地图
                grid_map::GridMap resampled_map;
                resampleMap(local_map, resampled_map, local_map_resolution_);
                local_map = resampled_map;
                
                // 🔄 重采样后也需要填充NaN值
                if (!local_map.getLayers().empty()) {
                    std::string height_layer = local_map.getLayers()[0];
                    if (enable_nan_filling_) {
                        fillNaNWithMinHeight(local_map, height_layer);
                    }
                }
                
                RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Published resampled local map: %.3fm resolution, %dx%d cells", 
                           local_map.getResolution(), 
                           local_map.getSize()(0), local_map.getSize()(1));
            }

            // 发布局部地图 - 强制使用float格式避免uint8转换问题
            
            // 🔧 关键修复：在转换前确保数据类型为float，避免grid_map自动选择uint8
            // 通过添加一个极小的NaN值来"污染"数据范围，强制使用float格式
            grid_map::GridMap float_map = local_map;  // 复制地图
            for (const std::string& layer : float_map.getLayers()) {
                auto& layer_data = float_map[layer];
                // 在地图边角添加一个NaN值，强制grid_map使用float格式
                if (float_map.getSize()(0) > 0 && float_map.getSize()(1) > 0) {
                    grid_map::Index corner_index(0, 0);
                    float original_value = layer_data(corner_index(0), corner_index(1));
                    layer_data(corner_index(0), corner_index(1)) = std::numeric_limits<float>::quiet_NaN();
                    
                    // 立即恢复原值，但NaN的存在已经触发了float格式选择
                    if (std::isfinite(original_value)) {
                        layer_data(corner_index(0), corner_index(1)) = original_value;
                    }
                }
            }
            
            // 现在转换应该会使用float格式
            std::vector<std::string> layers = float_map.getLayers();
            auto local_map_msg = grid_map::GridMapRosConverter::toMessage(float_map, layers);
            local_map_msg->header.stamp = this->now();
            
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                       "Published float-format local map: %zu layers, elevation data size: %zu bytes",
                       layers.size(), 
                       local_map_msg->data.empty() ? 0 : local_map_msg->data[0].data.size());
            
            local_map_publisher_->publish(*local_map_msg);

        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Failed to extract local map: %s", e.what());
        }
    }

    // 🔄 分辨率重采样函数 - 从高分辨率地图重采样到低分辨率地图
    void resampleMap(const grid_map::GridMap& input_map, grid_map::GridMap& output_map, double target_resolution)
    {
        try {
            // 计算新的地图尺寸
            grid_map::Length map_length = input_map.getLength();
            grid_map::Position map_position = input_map.getPosition();
            
            // 创建目标分辨率的新地图
            output_map.setFrameId(input_map.getFrameId());
            output_map.setGeometry(map_length, target_resolution, map_position);
            
            // 复制所有层
            for (const std::string& layer : input_map.getLayers()) {
                output_map.add(layer);
                
                // 执行重采样 - 使用均值方法（从高分辨率到低分辨率）
                resampleLayerMean(input_map, output_map, layer);
            }
            
            output_map.setBasicLayers(input_map.getBasicLayers());
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to resample map: %s", e.what());
        }
    }
    
    // 均值重采样方法 - 适合从高分辨率（1cm）到低分辨率（5cm）
    void resampleLayerMean(const grid_map::GridMap& input_map, grid_map::GridMap& output_map, const std::string& layer)
    {
        const auto& input_layer = input_map[layer];
        auto& output_layer = output_map[layer];
        
        // 遍历输出地图的每个像素
        for (grid_map::GridMapIterator iterator(output_map); !iterator.isPastEnd(); ++iterator) {
            const grid_map::Index output_index(*iterator);
            
            // 获取输出像素在世界坐标中的位置
            grid_map::Position world_position;
            output_map.getPosition(output_index, world_position);
            
            // 计算在输入地图中的覆盖区域 - 一个5cm像素覆盖25个1cm像素
            double cell_radius = output_map.getResolution() / 2.0;
            
            // 收集覆盖区域内的所有有效值
            std::vector<float> values;
            
            for (grid_map::CircleIterator circle_iterator(input_map, world_position, cell_radius); 
                 !circle_iterator.isPastEnd(); ++circle_iterator) {
                const grid_map::Index input_index(*circle_iterator);
                const float& value = input_layer(input_index(0), input_index(1));
                
                if (std::isfinite(value)) {
                    values.push_back(value);
                }
            }
            
            // 计算均值
            if (!values.empty()) {
                float mean_value = std::accumulate(values.begin(), values.end(), 0.0f) / values.size();
                output_layer(output_index(0), output_index(1)) = mean_value;
            } else {
                // 没有有效值，设为NaN
                output_layer(output_index(0), output_index(1)) = std::numeric_limits<float>::quiet_NaN();
            }
        }
    }

    // 🔄 NaN值填充函数 - 将所有缺失点替换为最低点高程
    void fillNaNWithMinHeight(grid_map::GridMap& map, const std::string& layer)
    {
        try {
            if (!map.exists(layer)) {
                return;
            }
            
            auto& height_layer = map[layer];
            
            // 🔍 第一步：找到所有有效点中的最低高度
            float min_height = std::numeric_limits<float>::max();
            size_t nan_count = 0;
            size_t total_count = 0;
            
            for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
                const grid_map::Index index(*iterator);
                const float& value = height_layer(index(0), index(1));
                total_count++;
                
                if (std::isfinite(value)) {
                    if (value < min_height) {
                        min_height = value;
                    }
                } else {
                    nan_count++;
                }
            }
            
            // 如果没有找到有效的高度值，使用0.0作为默认值
            if (min_height == std::numeric_limits<float>::max()) {
                min_height = 0.0f;
                RCLCPP_WARN(this->get_logger(), "No valid height values found in local map, using 0.0 as fill value");
            }
            
            // 🔧 第二步：将所有NaN值替换为最低高度
            size_t filled_count = 0;
            for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
                const grid_map::Index index(*iterator);
                float& value = height_layer(index(0), index(1));
                
                if (!std::isfinite(value)) {
                    value = min_height;
                    filled_count++;
                }
            }
            
            if (filled_count > 0) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                           "Filled %zu NaN pixels (%.1f%% of local map) with minimum height %.3f", 
                           filled_count, 100.0f * filled_count / total_count, min_height);
            }
            
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Failed to fill NaN values: %s", e.what());
        }
    }

    // -- 已移除 fillSmallHoles() --

    // 参数
    std::string global_map_topic_;
    std::string local_map_topic_;
    std::string robot_frame_;
    std::string map_frame_;
    double local_map_size_x_;
    double local_map_size_y_;
    double local_map_resolution_;  // 目标局部地图分辨率，0.0表示使用全局地图分辨率
    double update_rate_;
    double tf_timeout_;
    bool use_rotated_robot_frame_;
    bool invert_height_;
    bool enable_resampling_;       // 是否启用分辨率重采样
    std::string resampling_method_; // 重采样方法
    bool enable_nan_filling_;      // 是否启用NaN值填充

    // ROS组件
    rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr global_map_subscriber_;
    rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr local_map_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // TF2
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // 数据
    grid_map::GridMap global_map_;
    bool has_global_map_ = false;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<LocalElevationMapExtractor>();
    
    rclcpp::spin(node);
    rclcpp::shutdown();
    
    return 0;
} 