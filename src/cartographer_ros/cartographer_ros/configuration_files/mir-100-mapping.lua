-- Copyright 2018 The Cartographer Authors
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
  -- 设置SLAM算法要跟踪的ROS帧ID，一般为IMU的帧ID，如"imu_frame"
  tracking_frame = "imu_frame",
  -- 设置发布位姿的ROS帧ID，一般为底盘的帧ID，如"base_link"
  published_frame = "base_link",
  -- 设置cartographer提供的里程计的ROS帧ID，一般为"odom"
  odom_frame = "odom",
  -- 设置是否使用cartographer提供的里程计，如果为true，会发布odom_frame
  provide_odom_frame = true,
  -- 设置是否将发布的位姿投影到2D，如果为true，发布的位姿没有roll、pitch或z-offset
  publish_frame_projected_to_2d = false,
  use_odometry = true,
  use_pose_extrapolator = true,
  use_nav_sat = false,
  -- 设置是否使用地标数据，如果为true，会订阅/landmarks话题，并在SLAM过程中使用
  use_landmarks = true,
  -- 设置SLAM可以输入的/scan话题数目的最大值，每个话题对应一个激光雷达
  num_laser_scans = 2,
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
-- 设置是否将地标数据整合到轨迹中，如果为false，会将地标数据作为单独的轨迹
TRAJECTORY_BUILDER.collate_landmarks = false
-- 设置2D轨迹构建器的参数，其中num_accumulated_range_data表示每个submap需要累积的激光数据的数量
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 2
-- 设置是否使用IMU数据，如果为true，会订阅/imu话题，并在SLAM过程中使用
TRAJECTORY_BUILDER_2D.use_imu_data = true
-- 设置每个submap需要的激光数据的数量，值越大，submap越大
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 45

-- more points
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.max_length = 0.2
TRAJECTORY_BUILDER_2D.adaptive_voxel_filter.min_num_points = 400
-- slightly slower insertion
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.53
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.493
-- slightly shorter rays
TRAJECTORY_BUILDER_2D.max_range = 15.
-- wheel odometry is fine
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 20
-- IMU is ok
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 20

-- less outliers
POSE_GRAPH.constraint_builder.max_constraint_distance = 5.
POSE_GRAPH.constraint_builder.min_score = 0.5
-- tune down IMU in optimization
POSE_GRAPH.optimization_problem.acceleration_weight = 0.1 * 1e3
POSE_GRAPH.optimization_problem.rotation_weight = 0.1 * 3e5
-- ignore wheels in optimization
POSE_GRAPH.optimization_problem.odometry_translation_weight = 0.
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 0.
POSE_GRAPH.optimization_problem.log_solver_summary = true

return options

