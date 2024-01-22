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

-- 引入两个外部 Lua 文件，分别是 "map_builder.lua" 和 "trajectory_builder.lua"，它们包含了地图构建器和轨迹构建器的相关设置
include "map_builder.lua"
include "trajectory_builder.lua"

-- 定义配置选项
options = {
  map_builder = MAP_BUILDER,  -- 使用 "map_builder.lua" 中定义的 MAP_BUILDER
  trajectory_builder = TRAJECTORY_BUILDER,  -- 使用 "trajectory_builder.lua" 中定义的 TRAJECTORY_BUILDER
  map_frame = "map",  -- 地图坐标系的名称
  tracking_frame = "base_link",  -- 跟踪坐标系的名称
  published_frame = "base_link",  -- 发布坐标系的名称
  odom_frame = "odom",  -- 里程计坐标系的名称
  provide_odom_frame = true,  -- 是否提供 odom_frame
  publish_frame_projected_to_2d = false,  -- 是否发布 2D 投影的坐标系
  use_pose_extrapolator = true,  -- 是否使用位姿外推器
  use_odometry = false,  -- 是否使用里程计信息
  use_nav_sat = false,  -- 是否使用导航卫星信息
  use_landmarks = false,  -- 是否使用地标信息
  num_laser_scans = 0,  -- 使用的激光扫描数量
  num_multi_echo_laser_scans = 0,  -- 使用的多回波激光扫描数量
  num_subdivisions_per_laser_scan = 1,  -- 每个激光扫描的子分区数量
  num_point_clouds = 2,  -- 使用的点云数量
  lookup_transform_timeout_sec = 0.2,  -- 查找坐标变换的超时时间
  submap_publish_period_sec = 0.3,  -- 子图发布周期
  pose_publish_period_sec = 5e-3,  -- 位姿发布周期
  trajectory_publish_period_sec = 30e-3,  -- 轨迹发布周期
  rangefinder_sampling_ratio = 1.,  -- 激光测距仪的采样比率
  odometry_sampling_ratio = 1.,  -- 里程计的采样比率
  fixed_frame_pose_sampling_ratio = 1.,  -- 固定坐标系位姿的采样比率
  imu_sampling_ratio = 1.,  -- IMU 数据的采样比率
  landmarks_sampling_ratio = 1.,  -- 地标数据的采样比率
}

-- 在 TRAJECTORY_BUILDER_3D 中设置累积的激光测距数据数量
TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 160

-- 在 MAP_BUILDER 中启用 3D 轨迹构建器
MAP_BUILDER.use_trajectory_builder_3d = true

-- 设置 MAP_BUILDER 的后台线程数量
MAP_BUILDER.num_background_threads = 7

-- 配置 POSE_GRAPH 优化问题的 Huber Scale
POSE_GRAPH.optimization_problem.huber_scale = 5e2

-- 设置优化频率，每320个节点进行一次优化
POSE_GRAPH.optimize_every_n_nodes = 320

-- 配置 POSE_GRAPH 约束生成器的采样比率
POSE_GRAPH.constraint_builder.sampling_ratio = 0.03

-- 配置 POSE_GRAPH 优化问题的 Ceres Solver 选项，设置最大迭代次数
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 10

-- 配置 POSE_GRAPH 约束生成器的最小得分和全局定位的最小得分
POSE_GRAPH.constraint_builder.min_score = 0.62
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.66

-- 返回配置选项
return options

