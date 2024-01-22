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



include "map_builder.lua"
include "trajectory_builder.lua"


options = {

  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_frame", --dummy_base/imu_frame如果使用 IMU，它应该处于其位置，尽管它可能会旋转。常见的选择是“imu_link”。
  published_frame = "frame_link", --frame_link/dummy_base/odom 例如，如果“odom”帧由系统的不同部分提供，则为“odom”。在这种情况下，map_frame 中“odom”的姿势将被发布。否则，将其设置为“base_link”可能是合适的。
  odom_frame = "odom_2d",--仅当 Provide_odom_frame 为 true 时才使用。 published_frame和map_frame之间的帧用于发布（非循环闭合）本地SLAM结果。通常是“odom“”。
  provide_odom_frame = true, --true --如果启用，局部、非循环闭合、连续姿势将作为 map_frame 中的 odom_frame 发布。
  publish_frame_projected_to_2d = false, --/scan的坐标系是歪的，跟publish_frame_projected_to_2d设置为true有关系
  use_pose_extrapolator = true,  --使能位姿
  use_odometry = false,--是否使用里程计（底盘提供）
  use_nav_sat = false,  -- 设置是否使用GPS数据，如果为true，会订阅/fix话题，并在全局SLAM中使用
  use_landmarks = false,  -- 设置是否使用地标数据，如果为true，会订阅/landmarks话题，并在SLAM过程中使用
  num_laser_scans = 1, -- 0要订阅的激光扫描主题数量。订阅sensor_msgs/LaserScan 的“scan”主题（对于一台激光扫描仪）或主题“scan_1”、“scan_2”等（对于​​多台激光扫描仪）。
  num_multi_echo_laser_scans = 0,  --1要订阅的多回波激光扫描主题的数量。订阅sensor_msgs/MultiEchoLaserScan 的“echoes”主题（对于一台激光扫描仪）或主题“echoes_1”、“echoes_2”等（对于​​多台激光扫描仪）。
  num_subdivisions_per_laser_scan = 1, --10每扫描一帧作为一帧，原始值为10帧为一帧
  num_point_clouds = 0, --0订阅的点云主题数量。订阅一个测距仪的“points2”主题上的sensor_msgs/PointCloud2，或多个测距仪的主题“points2_1”、“points2_2”等。
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}


MAP_BUILDER.use_trajectory_builder_2d = true --对应map_builder.lua文件，这里设置成true.

TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1 --每4帧取一帧，防止雷达帧率太高耗费计算资源，可以减少计算量

-- 0改成0.10,比机器人半径小的都忽略
TRAJECTORY_BUILDER_2D.min_range = 0.10
-- 30改成3.5,限制在雷达最大扫描范围内，越小一般越精确些 6.0
TRAJECTORY_BUILDER_2D.max_range = 60.0
-- 5改成3,传感器数据超出有效范围最大值
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.
-- true改成false,不使用IMU数据，大家可以开启，然后对比下效果
TRAJECTORY_BUILDER_2D.use_imu_data = true
-- false改成true,使用实时回环检测来进行前端的扫描匹配
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true 
-- 1.0改成0.1,提高对运动的敏感度
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)

-- 0.55改成0.65,Fast csm的最低分数，高于此分数才进行优化。
POSE_GRAPH.constraint_builder.min_score = 0.65
--0.6改成0.7,全局定位最小分数，低于此分数则认为目前全局定位不准确
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7

-- 设置0可关闭全局SLAM
-- POSE_GRAPH.optimize_every_n_nodes = 0

-------------------------------------------------------------------------------------------------------------

-- MAP_BUILDER.use_trajectory_builder_2d = true

-- TRAJECTORY_BUILDER_2D.min_range = 0.05
-- TRAJECTORY_BUILDER_2D.max_range = 30
-- TRAJECTORY_BUILDER_2D.missing_data_ray_length = 8.5
-- TRAJECTORY_BUILDER_2D.use_imu_data = false
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 0.2
-- TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 5
-- TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
-- TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
-- TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 1.
-- TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 10
-- TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.2)
-- -- for current lidar only 1 is good value
-- TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1

-- TRAJECTORY_BUILDER_2D.min_z = -0.5
-- TRAJECTORY_BUILDER_2D.max_z = 0.5

-- POSE_GRAPH.constraint_builder.min_score = 0.65
-- POSE_GRAPH.constraint_builder.global_localization_min_score = 0.65
-- POSE_GRAPH.optimization_problem.huber_scale = 1e2
-- POSE_GRAPH.optimize_every_n_nodes = 30

return options

