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
  tracking_frame = "base_link", --如果使用 IMU，它应该处于其位置，尽管它可能会旋转。常见的选择是“imu_link”。
  published_frame = "base_link", --例如，如果“odom”帧由系统的不同部分提供，则为“odom”。在这种情况下，map_frame 中“odom”的姿势将被发布。否则，将其设置为“base_link”可能是合适的。
  odom_frame = "odom", --仅当 Provide_odom_frame 为 true 时才使用。 published_frame和map_frame之间的帧用于发布（非循环闭合）本地SLAM结果。通常是“odom“”。
  provide_odom_frame = true, --如果启用，局部、非循环闭合、连续姿势将作为 map_frame 中的 odom_frame 发布。
  publish_frame_projected_to_2d = false, --如果启用，发布的姿势将仅限于纯 2D 姿势（无滚动、俯仰或 z 偏移）。这可以防止由于姿势外推步骤而在 2D 模式下可能出现的潜在不需要的平面外姿势（例如，如果姿势应发布为类似“base-footprint”的帧）
  use_pose_extrapolator = true, --
  use_odometry = false, --如果启用，订阅主题“odom”上的 nav_msgs/Odometry。在这种情况下必须提供里程计，并且该信息将包含在SLAM中。
  use_nav_sat = false, --如果启用，则订阅主题“修复”的sensor_msgs/NavSatFix。在这种情况下必须提供导航数据，并且该信息将包含在全局SLAM中。
  use_landmarks = false, --如果启用，则订阅有关“地标”主题的 cartographer_ros_msgs/LandmarkList。必须提供地标，
  num_laser_scans = 0, --要订阅的激光扫描主题数量。订阅sensor_msgs/LaserScan 的“scan”主题（对于一台激光扫描仪）或主题“scan_1”、“scan_2”等（对于​​多台激光扫描仪）。
  num_multi_echo_laser_scans = 1, --要订阅的多回波激光扫描主题的数量。订阅sensor_msgs/MultiEchoLaserScan 的“echoes”主题（对于一台激光扫描仪）或主题“echoes_1”、“echoes_2”等（对于​​多台激光扫描仪）。
  num_subdivisions_per_laser_scan = 10, 
  num_point_clouds = 0, --订阅的点云主题数量。订阅一个测距仪的“points2”主题上的sensor_msgs/PointCloud2，或多个测距仪的主题“points2_1”、“points2_2”等。
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1., --IMU 消息的固定比率采样。
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 10

return options
