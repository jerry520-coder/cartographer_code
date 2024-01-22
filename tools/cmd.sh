#!/bin/bash

#常用命令
ros2 bag play 0115_0.db3 --topics /argus_ros2/imu_result /lidar_points --remap /lidar_points:=/points /argus_ros2/imu_result:=/imu/data
ros2 bag record -o 0117 /scan /lidar_points /odom /argus_ros2/imu_result /tf /tf_static
ros2 run tf2_ros static_transform_publisher 0.0275741 -0.903559 -0.356313 -0.00233817 0.711001 -0.703177 0.00386281 hesai_lidar imu_link
./tools/run.sh fm | grep bag
ros2 service call /finish_trajectory cartographer_ros_msgs/srv/finish_trajectory "{trajectory_id: 0}"
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '/home/binghe/orbbec_code/cartographer_ros2/map/map_B2_0118.pbstream'}"
ros2 run nav2_map_server map_saver_cli -t map -f fishbot_map
#   -t <map_topic>
#   -f <mapname>
