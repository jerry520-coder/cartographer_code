-- Copyright 2016 The Cartographer Authors
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
--      http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

-- 引入map_builder.lua文件，它定义了MAP_BUILDER对象，用来设置地图构建的选项
include "map_builder.lua"
-- 引入trajectory_builder.lua文件，它定义了TRAJECTORY_BUILDER对象，用来设置轨迹构建的选项
include "trajectory_builder.lua"

-- 定义一个名为options的表，用来存储所有的参数和选项
options = {
  -- 设置map_builder的选项，它是一个MAP_BUILDER对象
  map_builder = MAP_BUILDER,
  -- 设置trajectory_builder的选项，它是一个TRAJECTORY_BUILDER对象
  trajectory_builder = TRAJECTORY_BUILDER,
  -- 设置地图的ROS帧ID，一般为"map"
  map_frame = "map",
  -- 设置SLAM算法要跟踪的ROS帧ID，一般为底盘的帧ID，如"base_footprint"
  tracking_frame = "base_footprint",
  -- 设置发布位姿的ROS帧ID，一般为底盘的帧ID，如"base_footprint"
  published_frame = "base_footprint",
  -- 设置cartographer提供的里程计的ROS帧ID，一般为"odom"
  odom_frame = "odom",
  -- 设置是否使用cartographer提供的里程计，如果为true，会发布odom_frame
  provide_odom_frame = true,
  -- 设置是否将发布的位姿投影到2D，如果为true，发布的位姿没有roll、pitch或z-offset
  publish_frame_projected_to_2d = false,
  -- 设置是否使用位姿外推器，如果为true，会根据最近的位姿和传感器数据预测当前的位姿
  use_pose_extrapolator = true,
  -- 设置是否使用里程计数据，如果为false，不会订阅/odom话题
  use_odometry = false,
  -- 设置是否使用GPS数据，如果为false，不会订阅/fix话题
  use_nav_sat = false,
  -- 设置是否使用地标数据，如果为false，不会订阅/landmarks话题
  use_landmarks = false,
  -- 设置SLAM可以输入的/scan话题数目的最大值，每个话题对应一个激光雷达
  num_laser_scans = 1,
  -- 设置SLAM可以输入的sensor_msgs/MultiEchoLaserScan话题数目的最大值，每个话题对应一个多回波激光雷达
  num_multi_echo_laser_scans = 0,
  -- 设置将每个接收到的（multi_echo）激光scan分割成的点云数，细分scan可以在扫描仪移动时取消扫描畸变
  num_subdivisions_per_laser_scan = 1,
  -- 设置SLAM可以输入的sensor_msgs/PointCloud2话题数目的最大值，每个话题对应一个点云传感器
  num_point_clouds = 0,
  -- 设置使用tf2查找transform的超时时间（秒）
  lookup_transform_timeout_sec = 0.2,
  -- 设置发布submap的时间间隔（秒）
  submap_publish_period_sec = 0.3,
  -- 设置发布位姿的时间间隔（秒），值为5e-3的时候为200Hz
  pose_publish_period_sec = 5e-3,
  -- 设置发布轨迹节点的时间间隔（秒），值为30e-3的时候为30ms
  trajectory_publish_period_sec = 30e-3,
  -- 设置测距仪的固定采样比率，值为1.0表示不采样，值为0.5表示每两个点采样一个
  rangefinder_sampling_ratio = 1.,
  -- 设置里程计的固定采样比率，值为1.0表示不采样，值为0.5表示每两个点采样一个
  odometry_sampling_ratio = 1.,
  -- 设置固定帧位姿的固定采样比率，值为1.0表示不采样，值为0.5表示每两个点采样一个
  fixed_frame_pose_sampling_ratio = 1.,
  -- 设置IMU的固定采样比率，值为1.0表示不采样，值为0.5表示每两个点采样一个
  imu_sampling_ratio = 1.,
  -- 设置地标的固定采样比率，值为1.0表示不采样，值为0.5表示每两个点采样一个
  landmarks_sampling_ratio = 1.,
}

-- 设置是否使用2D的轨迹构建器，如果为true，会使用TRAJECTORY_BUILDER_2D对象
MAP_BUILDER.use_trajectory_builder_2d = true

-- 设置2D轨迹构建器的参数
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true -- 设置是否使用在线相关匹配，如果为true，会在插入submap之前进行匹配，可以提高精度，但会降低速度
TRAJECTORY_BUILDER_2D.use_imu_data = false -- 设置是否使用IMU数据，如果为false，不会订阅/imu话题
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.15 -- 设置在线相关匹配的线性搜索窗口，单位为米，值越大，匹配范围越大，但速度越慢
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(35.) -- 设置在线相关匹配的角度搜索窗口，单位为弧度，值越大，匹配范围越大，但速度越慢

-- 设置位姿图优化问题的参数
POSE_GRAPH.optimization_problem.huber_scale = 1e2 -- 设置Huber损失函数的缩放系数，值越大，对离群点的惩罚越小

-- 返回options表，供cartographer使用
return options

