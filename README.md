```bash
https://github.com/Evanyhz/elevation_mapping_bxi6

一、 编译kindr，native C++ build：                     
        cd src/kindr && mkdir build && cd build  #若显示已有build目录，则把原来的删除即可
        cmake .. -DUSE_CMAKE=true
        sudo make install

二、编译&安装 kindr_ros

         cd xx_ws #回到 _ws 工作目录
         source /opt/ros/humble/setup.bash
         colcon build --packages-up-to kindr_ros

三、编译 grid_map

         #安装相关依赖：
         sudo apt update && sudo apt install -y \
                        ros-humble-filters \
                        ros-humble-pcl-ros \
                        ros-humble-nav2-costmap-2d \
                        ros-humble-navigation2 \
                        ros-humble-octomap-msgs \
                        liboctomap-dev \
                        python3-ament-package

        cd src
        git clone https://github.com/anybotics/grid_map.git --branch humble

四、全部编译：
kindr_ros的galactic分支与ROS2 Humble的头文件路径结构不兼容std_msgs、builtin_interfaces、geometry_msgs等ROS接口的路径在galactic和humble之间发生了变化
解决方案：使用符号链接方法解决路径兼容性问题：
# 创建的符号链接：
sudo ln -sf /opt/ros/humble/include/std_msgs/std_msgs /usr/local/include/std_msgs
sudo ln -sf /opt/ros/humble/include/builtin_interfaces/builtin_interfaces /usr/local/include/builtin_interfaces  
sudo ln -sf /opt/ros/humble/include/geometry_msgs/geometry_msgs /usr/local/include/geometry_msgs
sudo ln -sf /home/<你的用户名>/elevation_mapping_bxi/install/kindr/include/kindr/kindr /usr/local/include/kindr

        cd ../
        colcon build --symlink-install

#至此，编译完成

四、启用高程图：（单独节点启动方式）
        colcon build --packages-select elevation_mapping
        # 全局高程图：
        source install/setup.bash && ros2 launch elevation_mapping global_elevation_map_extractor_launch.py 
        # 局部高程图：
        source install/setup.bash && ros2 launch elevation_mapping local_elevation_map_extractor_z_up_launch.py
        #局部高程图转高度矩阵：
        source /opt/ros/humble/setup.bash && source install/setup.bash && python3 simple_robot_height_map_publisher.py

         #ros2 bag record /robot_height_map

五、高程图参数修改：
   1.全局高程图的相关文件在/elevation_mapping/launch/global_elevation_mapping_extractor_launch.py
   2.局部高程图：/elevation_mapping/launch/local_elevation_mapping_extractor_z_up_launch.py
               && /elevation_mapping/src/local_elevation_mapping_extractor.cpp
   3.局部高程图转 一维数组：simple_robot_height_map_publisher.py

   注：此局部高程图测量的都是相对于mid360的imu的高度，测量范围为以机器人为中心的 “1.2x0.4m” 的矩形范围。可以参考手册中imu在雷达坐标系下的位置来消除高度差，本方案已经消除了x,y方向的偏差（因全局高程图是以imu的初始化坐标为参考的，故高度值也是相对imu的；而局部高程图是从全局高程图中提取的，故无法直接消除z偏差），只需要关注z坐标。如本方案中，雷达是倒置的，imu位于lidar上方 0.04412m处 ，只需将得到的高度矩阵全部 +0.04412m，即可得到相对于lidar坐标系的高度值。

 
<！--
-->
```
