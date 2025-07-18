```bash
       git clone https://github.com/Evanyhz/elevation_mapping_bxi4

一、 编译kindr，它是一个native C++ build：                     
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
全部编译：
        cd ../
        colcon build --symlink-install

四、启用高程图：
        colcon build --packages-select elevation_mapping
        # 全局高程图：
        source install/setup.bash && ros2 launch elevation_mapping global_elevation_map_extractor_launch.py 
        # 局部高程图：
        source install/setup.bash && ros2 launch elevation_mapping local_elevation_map_extractor_z_up_launch.py
        #局部高程图转高度矩阵：
        source /opt/ros/humble/setup.bash && source install/setup.bash && python3 simple_robot_height_map_publisher.py


         ros2 bag record /robot_height_map 









```
kindr_ros的galactic分支与ROS2 Humble的头文件路径结构不兼容
std_msgs、builtin_interfaces、geometry_msgs等ROS接口的路径在galactic和humble之间发生了变化
解决方案：
使用符号链接方法成功解决了路径兼容性问题：
# 创建的符号链接：
sudo ln -sf /opt/ros/humble/include/std_msgs/std_msgs /usr/local/include/std_msgs
sudo ln -sf /opt/ros/humble/include/builtin_interfaces/builtin_interfaces /usr/local/include/builtin_interfaces  
sudo ln -sf /opt/ros/humble/include/geometry_msgs/geometry_msgs /usr/local/include/geometry_msgs
sudo ln -sf /home/<你的用户名>/elevation_mapping_bxi/install/kindr/include/kindr/kindr /usr/local/include/kindr
