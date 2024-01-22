#!/bin/bash

ros2 bag play /home/binghe/file_mount_point/0118/0118/0118_0.db3 \
    --topics /argus_ros2/imu_result /lidar_points \
    --remap /lidar_points:=/points /argus_ros2/imu_result:=/imu/data \
    --clock

# 节点没有足够的缓冲区来接收话题的消息。这可能会导致您的节点丢失一些消息，或者延迟处理消息。
# 这种情况通常发生在您的节点订阅了一个高频率或者大数据量的话题，而您的计算机性能不足以及时处理这些消息
# ros2 bag play /home/binghe/file_mount_point/0118/0118/0118_0.db3 \
#     --topics /argus_ros2/imu_result /lidar_points /scan /odom -l \
#     --clock \
#     -r 2.0 \
#     --read-ahead-queue-size 1000

# ros2 bag play /home/binghe/file_mount_point/0118/0118/0118_0.db3 \
#     --topics /argus_ros2/imu_result /lidar_points /scan /odom -l \
#     --clock
