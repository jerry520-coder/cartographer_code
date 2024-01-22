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

-- 定义一个名为TRAJECTORY_BUILDER_2D的表，用来存储2D轨迹构建器的参数和选项
TRAJECTORY_BUILDER_2D = {
  -- 设置是否使用IMU数据，如果为true，会根据IMU数据进行运动补偿和位姿推算
  use_imu_data = true,
  -- 设置激光雷达的最小有效距离，单位为米，低于这个距离的点会被忽略
  min_range = 0.,
  -- 设置激光雷达的最大有效距离，单位为米，超过这个距离的点会被忽略
  max_range = 30.,
  -- 设置激光雷达的最小有效高度，单位为米，低于这个高度的点会被忽略
  min_z = -0.8,
  -- 设置激光雷达的最大有效高度，单位为米，超过这个高度的点会被忽略
  max_z = 2.,
  -- 设置激光雷达的缺失数据的射线长度，单位为米，如果激光雷达的某个角度没有返回数据，会认为该方向有一条长度为该值的射线
  missing_data_ray_length = 5.,
  -- 设置激光雷达的累积数据的数量，值为1表示每次只使用一帧激光数据，值大于1表示每次使用多帧激光数据的平均值
  num_accumulated_range_data = 1,
  -- 设置体素滤波器的大小，单位为米，值越小，滤波效果越弱，保留的点越多
  voxel_filter_size = 0.025,

  -- 设置自适应体素滤波器的选项，它用来对激光数据进行降采样，减少数据量
  adaptive_voxel_filter = {
    -- 设置体素滤波器的最大长度，单位为米，值越大，滤波效果越弱，保留的点越多
    max_length = 0.5,
    -- 设置体素滤波器的最小点数，低于这个点数的体素会被丢弃
    min_num_points = 200,
    -- 设置体素滤波器的最大距离，单位为米，超过这个距离的点会被丢弃
    max_range = 50.,
  },

  -- 设置闭环检测时的自适应体素滤波器的选项，它用来对激光数据进行降采样，减少数据量
  loop_closure_adaptive_voxel_filter = {
    -- 设置体素滤波器的最大长度，单位为米，值越大，滤波效果越弱，保留的点越多
    max_length = 0.9,
    -- 设置体素滤波器的最小点数，低于这个点数的体素会被丢弃
    min_num_points = 100,
    -- 设置体素滤波器的最大距离，单位为米，超过这个距离的点会被丢弃
    max_range = 50.,
  },

  -- 设置是否使用在线相关匹配，如果为true，会在每次插入新的激光数据时进行相关匹配，提高定位精度，但消耗资源更多
  use_online_correlative_scan_matching = false,
  -- 设置实时相关匹配器的选项，它用来在每次插入新的激光数据时进行相关匹配，提高定位精度
  real_time_correlative_scan_matcher = {
    -- 设置线性搜索窗口，单位为米，值越大，匹配范围越大，但速度越慢
    linear_search_window = 0.1,
    -- 设置角度搜索窗口，单位为弧度，值越大，匹配范围越大，但速度越慢
    angular_search_window = math.rad(20.),
    -- 设置平移误差的权重，值越大，平移误差的惩罚越大
    translation_delta_cost_weight = 1e-1,
    -- 设置旋转误差的权重，值越大，旋转误差的惩罚越大
    rotation_delta_cost_weight = 1e-1,
  },

  -- 设置ceres扫描匹配器的选项，它用来在每次插入新的激光数据时进行ceres优化，提高定位精度
  ceres_scan_matcher = {
    -- 设置占据空间的权重，值越大，占据空间误差的惩罚越大
    occupied_space_weight = 1.,
    -- 设置平移的权重，值越大，平移误差的惩罚越大
    translation_weight = 10.,
    -- 设置旋转的权重，值越大，旋转误差的惩罚越大
    rotation_weight = 40.,
    -- 设置ceres求解器的选项，它用来控制ceres优化的过程
    ceres_solver_options = {
      -- 设置是否使用非单调的步长，如果为false，可能会保证收敛，但也可能会降低速度
      use_nonmonotonic_steps = false,
      -- 设置最大迭代次数，值越大，优化时间越长，但精度可能会更高
      max_num_iterations = 20,
      -- 设置使用的线程数，值越大，优化速度越快，但消耗资源越多
      num_threads = 1,
    },
  },

  -- 设置运动滤波器的选项，它用来对激光数据进行滤波，减少数据量
  motion_filter = {
    -- 设置运动滤波器的最大时间间隔，单位为秒，超过这个时间间隔的数据会被保留
    max_time_seconds = 5.,
    -- 设置运动滤波器的最大距离间隔，单位为米，超过这个距离间隔的数据会被保留
    max_distance_meters = 0.2,
    -- 设置运动滤波器的最大角度间隔，单位为弧度，超过这个角度间隔的数据会被保留
    max_angle_radians = math.rad(1.),
  },

  -- TODO(schwoere,wohe): Remove this constant. This is only kept for ROS.
  -- 设置IMU的重力时间常数，单位为秒，值越大，对重力的估计越平滑，但对运动的响应越慢
  imu_gravity_time_constant = 10.,
  -- 设置位姿外推器的选项，它用来根据IMU数据和里程计数据进行位姿推算
  pose_extrapolator = {
    -- 设置是否使用基于IMU的位姿外推器，如果为false，会使用基于常速度的位姿外推器
    use_imu_based = false,
    -- 设置基于常速度的位姿外推器的选项，它用来根据IMU数据和里程计数据进行位姿推算
    constant_velocity = {
      -- 设置IMU的重力时间常数，单位为秒，值
      imu_gravity_time_constant = 10., -- 这个参数用于控制IMU的重力补偿，数值越大，表示对重力的信任度越高，数值越小，表示对重力的信任度越低
      pose_queue_duration = 0.001, -- 这个参数用于控制IMU的姿态数据的缓存时间，单位为秒，数值越大，表示缓存的数据越多，数值越小，表示缓存的数据越少
    },
    imu_based = {-- 这个部分是基于IMU的位姿优化的参数设置
      pose_queue_duration = 5., -- 这个参数用于控制优化时使用的IMU的姿态数据的时间窗口，单位为秒，数值越大，表示使用的数据越多，数值越小，表示使用的数据越少
      gravity_constant = 9.806, -- 这个参数用于设置重力加速度的常数，单位为m/s^2，一般取当地的重力加速度值
      pose_translation_weight = 1., -- 这个参数用于设置位姿的平移部分的权重，数值越大，表示对平移的约束越强，数值越小，表示对平移的约束越弱
      pose_rotation_weight = 1., -- 这个参数用于设置位姿的旋转部分的权重，数值越大，表示对旋转的约束越强，数值越小，表示对旋转的约束越弱
      imu_acceleration_weight = 1., -- 这个参数用于设置IMU的加速度数据的权重，数值越大，表示对加速度的信任度越高，数值越小，表示对加速度的信任度越低
      imu_rotation_weight = 1., -- 这个参数用于设置IMU的角速度数据的权重，数值越大，表示对角速度的信任度越高，数值越小，表示对角速度的信任度越低
      odometry_translation_weight = 1., -- 这个参数用于设置里程计的平移数据的权重，数值越大，表示对里程计的平移的信任度越高，数值越小，表示对里程计的平移的信任度越低
      odometry_rotation_weight = 1., -- 这个参数用于设置里程计的旋转数据的权重，数值越大，表示对里程计的旋转的信任度越高，数值越小，表示对里程计的旋转的信任度越低
      solver_options = { -- 这个部分是优化求解器的参数设置
        use_nonmonotonic_steps = false; -- 这个参数用于控制是否使用非单调步长，如果为true，表示允许优化过程中目标函数的值增加，如果为false，表示只允许目标函数的值减少
        max_num_iterations = 10; -- 这个参数用于设置优化的最大迭代次数，数值越大，表示优化的时间越长，数值越小，表示优化的时间越短
        num_threads = 1; -- 这个参数用于设置优化的线程数，数值越大，表示优化的并行度越高，数值越小，表示优化的并行度越低
      },
    },
  },

  submaps = {-- 这个部分是子地图的参数设置
    num_range_data = 90, -- 这个参数用于设置每个子地图包含的激光数据的数量，数值越大，表示每个子地图的范围越大，数值越小，表示每个子地图的范围越小
    grid_options_2d = { -- 这个部分是二维栅格地图的参数设置
      grid_type = "PROBABILITY_GRID", -- 这个参数用于设置栅格地图的类型，可以是概率栅格或者TSDF栅格
      resolution = 0.05, -- 这个参数用于设置栅格地图的分辨率，单位为米，数值越大，表示栅格的大小越大，数值越小，表示栅格的大小越小
    },
    range_data_inserter = {-- 这个部分是激光数据插入器的参数设置
        range_data_inserter_type = "PROBABILITY_GRID_INSERTER_2D", -- 这个参数用于设置激光数据插入器的类型，可以是概率栅格插入器或者TSDF栅格插入器
        probability_grid_range_data_inserter = { -- 这个部分是概率栅格插入器的参数设置
          insert_free_space = true, -- 这个参数用于控制是否在激光数据的空闲区域插入空闲概率，如果为true，表示插入空闲概率，如果为false，表示不插入空闲概率
          hit_probability = 0.55, -- 这个参数用于设置激光数据的命中概率，数值越大，表示对激光数据的信任度越高，数值越小，表示对激光数据的信任度越低
          miss_probability = 0.49, -- 这个参数用于设置激光数据的未命中概率，数值越大，表示对激光数据的否定程度越高，数值越小，表示对激光数据的否定程度越低
      },
      tsdf_range_data_inserter = {-- 这个部分是TSDF栅格插入器的参数设置
        truncation_distance = 0.3, -- 这个参数用于设置TSDF栅格的截断距离，单位为米，数值越大，表示TSDF栅格的厚度越大，数值越小，表示TSDF栅格的厚度越小
        maximum_weight = 10., -- 这个参数用于设置TSDF栅格的最大权重，数值越大，表示TSDF栅格的更新速度越慢，数值越小，表示TSDF栅格的更新速度越快
        update_free_space = false, -- 这个参数用于控制是否在激光数据的空闲区域更新TSDF栅格的值，如果为true，表示更新TSDF栅格的值，如果为false，表示不更新TSDF栅格的值
        normal_estimation_options = { -- 这个部分是法向量估计的参数设置
          num_normal_samples = 4, -- 这个参数用于设置法向量估计时使用的邻域点的数量，数值越大，表示法向量估计的精度越高，数值越小，表示法向量估计的精度越低
          sample_radius = 0.5, -- 这个参数用于设置法向量估计时使用的邻域点的半径，单位为米，数值越大，表示法向量估计的范围越大，数值越小，表示法向量估计的范围越小
        },
        project_sdf_distance_to_scan_normal = true, -- 这个参数用于控制是否将TSDF栅格的距离值投影到激光数据的法向量上，如果为true，表示投影到法向量上，如果为false，表示不投影到法向量上
        update_weight_range_exponent = 0, -- 这个参数用于设置TSDF栅格的更新权重
        update_weight_angle_scan_normal_to_ray_kernel_bandwidth = 0.5,
        update_weight_distance_cell_to_hit_kernel_bandwidth = 0.5,
      },
    },
  },
}
