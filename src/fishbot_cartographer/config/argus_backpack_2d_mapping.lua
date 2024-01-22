include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "imu_frame", --sensor_to_tracking->translation().norm() < 1e-5 IMU 帧必须与跟踪帧同位。否则，将线性加速度转换到跟踪帧将是不精确的。
  -- base_link改为odom,发布map到odom之间的位姿态。 就算设置为odom，/scan的坐标系也是歪的
  published_frame = "frame_link", ----frame_link/dummy_base/odom 用作发布姿势的子框架的 ROS 框架 ID。例如，如果“odom”帧由系统的不同部分提供，则为“odom”。在这种情况下，map_frame 中“odom”的位姿将被发布。否则，将其设置为“base_link”可能是合适的。
  odom_frame = "odom_2d",
  -- true改为false，不用提供里程计数据
  provide_odom_frame = true,
  -- false改为true，仅发布2D位资
  publish_frame_projected_to_2d = false, --/scan的坐标系是歪的，跟publish_frame_projected_to_2d设置为true有关系
  -- false改为true，使用里程计数据
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  -- 0改为1,使用一个雷达
  num_laser_scans = 1,
  -- 1改为0，不使用多波雷达
  num_multi_echo_laser_scans = 0,
  -- 10改为1，1/1=1等于不分割
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
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


-- false改为true，启动2D SLAM
MAP_BUILDER.use_trajectory_builder_2d = true

-- 0改成0.10,比机器人半径小的都忽略
TRAJECTORY_BUILDER_2D.min_range = 0.10
-- 30改成3.5,限制在雷达最大扫描范围内，越小一般越精确些 6.0
TRAJECTORY_BUILDER_2D.max_range = 200.0
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

return options
