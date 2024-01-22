
# 1，在线建图

拷贝修改文件为自己对应的设置：
backpack_2d.launch -> my_robot_2d.launch
backpack_2d.lua -> my_robot_2d.lua

在.lua配置文件中设定TF转换关系：

map_frame = "map",
tracking_frame = "base_link",
published_frame = "odom",
odom_frame = "odom",
启动数据发布：
sensor_msgs/LaserScan，nav_msgs/Odometry，sensor_msgs/Imu等

启动建图：
```bash
roslaunch cartographer_ros my_robot_2d.launch 
```

建图完成后保存地图：
```bash
rosservice call /finish_trajectory 0
rosservice call /write_state "{filename: '${HOME}/Downloads/MyMap.pbstream'}"

rosrun cartographer_ros cartographer_pbstream_to_ros_map  -map_filestem=${HOME}/Downloads/MyMap  -pbstream_filename=${HOME}/Downloads/MyMap.pbstream -resolution=0.05
```

# 2，离线建图
拷贝修改文件为自己对应的设置：
backpack_2d.launch -> offline_robot_2d.launch
backpack_2d.lua -> my_robot_2d.lua

启动建图：

roslaunch cartographer_ros offline_robot_2d.launch bag_filenames:=${HOME}/Downloads/MyData.bag
其他同1

# 3，在线定位
拷贝修改文件为自己对应的设置：
demo_backpack_2d_localization.launch -> my_robot_localization_2d.launch
backpack_2d_localization.lua -> my_robot_localization_2d.lua

在.lua配置文件中打开定位功能：

TRAJECTORY_BUILDER.pure_localization_trimmer = {
    max_submaps_to_keep = 2,
}
启动数据发布：

sensor_msgs/LaserScan，nav_msgs/Odometry，sensor_msgs/Imu等

开始定位：

roslaunch cartographer_ros my_robot_localization_2d.launch 


# 4，离线定位
拷贝修改文件为自己对应的设置：
demo_backpack_2d_localization.launch -> offline_robot_localization_2d.launch
backpack_2d_localization.lua -> my_robot_localization_2d.lua

开始定位：

roslaunch cartographer_ros offline_robot_localization_2d.launch ${HOME}/Downloads/LocalizationData.bag