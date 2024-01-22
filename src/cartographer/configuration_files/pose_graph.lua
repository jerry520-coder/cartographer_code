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

-- 定义一个名为POSE_GRAPH的表，用来存储位姿图优化的参数和选项
POSE_GRAPH = {
  -- 设置每多少个节点进行一次优化，值为0表示关闭后端优化
  optimize_every_n_nodes = 90,
  -- 设置约束生成器的选项，它用来寻找闭环约束和子图间约束
  constraint_builder = {
    -- 设置约束采样比率，值为0.3表示每三个候选约束采样一个
    sampling_ratio = 0.3,
    -- 设置约束的最大距离，单位为米，超过这个距离的候选约束会被忽略
    max_constraint_distance = 15.,
    -- 设置约束的最小得分，低于这个得分的候选约束会被忽略
    min_score = 0.55,
    -- 设置全局定位的最小得分，低于这个得分的全局定位结果会被忽略
    global_localization_min_score = 0.6,
    -- 设置闭环约束的平移权重，值越大，平移误差的惩罚越大
    loop_closure_translation_weight = 1.1e4,
    -- 设置闭环约束的旋转权重，值越大，旋转误差的惩罚越大
    loop_closure_rotation_weight = 1e5,
    -- 设置是否打印匹配的日志信息，如果为true，会输出匹配的结果和得分
    log_matches = true,
    -- 设置快速相关匹配器的选项，它用来在2D模式下进行相关匹配，加速约束的寻找
    fast_correlative_scan_matcher = {
      -- 设置线性搜索窗口，单位为米，值越大，匹配范围越大，但速度越慢
      linear_search_window = 7.,
      -- 设置角度搜索窗口，单位为弧度，值越大，匹配范围越大，但速度越慢
      angular_search_window = math.rad(30.),
      -- 设置分支定界的深度，值越大，匹配精度越高，但速度越慢
      branch_and_bound_depth = 7,
    },
    -- 设置ceres扫描匹配器的选项，它用来在2D模式下进行ceres优化，提高约束的精度
    ceres_scan_matcher = {
      -- 设置占据空间的权重，值越大，占据空间误差的惩罚越大
      occupied_space_weight = 20.,
      -- 设置平移的权重，值越大，平移误差的惩罚越大
      translation_weight = 10.,
      -- 设置旋转的权重，值越大，旋转误差的惩罚越大
      rotation_weight = 1.,
      -- 设置ceres求解器的选项，它用来控制ceres优化的过程
      ceres_solver_options = {
        -- 设置是否使用非单调的步长，如果为true，可能会加速收敛，但也可能会陷入局部最优
        use_nonmonotonic_steps = true,
        -- 设置最大迭代次数，值越大，优化时间越长，但精度可能会更高
        max_num_iterations = 10,
        -- 设置使用的线程数，值越大，优化速度越快，但消耗资源越多
        num_threads = 1,
      },
    },
    -- 设置快速相关匹配器的选项，它用来在3D模式下进行相关匹配，加速约束的寻找
    fast_correlative_scan_matcher_3d = {
      -- 设置分支定界的深度，值越大，匹配精度越高，但速度越慢
      branch_and_bound_depth = 8,
      -- 设置全分辨率的深度，值越大，匹配精度越高，但速度越慢
      full_resolution_depth = 3,
      -- 设置最小的旋转得分，低于这个得分的候选匹配会被忽略
      min_rotational_score = 0.77,
      -- 设置最小的低分辨率得分，低于这个得分的候选匹配会被忽略
      min_low_resolution_score = 0.55,
      -- 设置线性XY搜索窗口，单位为米，值越大，匹配范围越大，但速度越慢
      linear_xy_search_window = 5.,
      -- 设置线性Z搜索窗口，单位为米，值越大，匹配范围越大，但速度越慢
      linear_z_search_window = 1.,
      -- 设置角度搜索窗口，单位为弧度，值越大，匹配范围越大，但速度越慢
      angular_search_window = math.rad(15.),
    },
    -- 设置ceres扫描匹配器的选项，它用来在3D模式下进行ceres优化，提高约束的精度
    ceres_scan_matcher_3d = {
      -- 设置占据空间的权重，值越大，占据空间误差的惩罚越大
      occupied_space_weight_0 = 5.,
      -- 设置占据空间的权重，值越大，占据空间误差的惩罚越大
      occupied_space_weight_1 = 30.,
      -- 设置平移的权重，值越大，平移误差的惩罚越大
      translation_weight = 10.,
      -- 设置旋转的权重，值越大，旋转误差的惩罚越大
      rotation_weight = 1.,
      -- 设置是否只优化偏航角，如果为false，会优化所有的欧拉角
      only_optimize_yaw = false,
      -- 设置ceres求解器的选项，它用来控制ceres优化的过程
      ceres_solver_options = {
        -- 设置是否使用非单调的步长，如果为false，可能会保证收敛，但也可能会降低速度
        use_nonmonotonic_steps = false,
        -- 设置最大迭代次数，值越大，优化时间越长，但精度可能会更高
        max_num_iterations = 10,
        -- 设置使用的线程数，值越大，优化速度越快，但消耗资源越多
        num_threads = 1,
      },
    },
  },
  -- 设置匹配器的平移权重，值越大，平移误差的惩罚越大
  matcher_translation_weight = 5e2,
  -- 设置匹配器的旋转权重，值越大，旋转误差的惩罚越大
  matcher_rotation_weight = 1.6e3,
  -- 设置优化问题的选项，它用来控制优化的目标函数和参数
  optimization_problem = {
    -- 设置Huber损失函数的缩放系数，值越大，对离群点的惩罚越小
    huber_scale = 1e1,
    -- 设置加速度的权重，值越大，加速度误差的惩罚越大
    acceleration_weight = 1.1e2,
    -- 设置旋转的权重，值越大，旋转
    rotation_weight = 1.6e4, -- 旋转权重，表示旋转误差的代价，值越大，旋转误差越小
    local_slam_pose_translation_weight = 1e5, -- 本地SLAM位姿平移权重，表示平移误差的代价，值越大，平移误差越小
    local_slam_pose_rotation_weight = 1e5, -- 本地SLAM位姿旋转权重，表示旋转误差的代价，值越大，旋转误差越小
    odometry_translation_weight = 1e5, -- 里程计平移权重，表示里程计平移误差的代价，值越大，里程计平移误差越小
    odometry_rotation_weight = 1e5, -- 里程计旋转权重，表示里程计旋转误差的代价，值越大，里程计旋转误差越小
    fixed_frame_pose_translation_weight = 1e1, -- 固定帧位姿平移权重，表示固定帧平移误差的代价，值越大，固定帧平移误差越小
    fixed_frame_pose_rotation_weight = 1e2, -- 固定帧位姿旋转权重，表示固定帧旋转误差的代价，值越大，固定帧旋转误差越小
    fixed_frame_pose_use_tolerant_loss = false, -- 是否使用容忍损失函数，如果为true，表示对于大的误差，不会惩罚太多，可以避免过拟合
    fixed_frame_pose_tolerant_loss_param_a = 1, -- 容忍损失函数的参数a，表示误差的阈值，误差小于该值时，使用二次函数，误差大于该值时，使用线性函数
    fixed_frame_pose_tolerant_loss_param_b = 1, -- 容忍损失函数的参数b，表示线性函数的斜率，值越大，表示对于大的误差，惩罚越多
    log_solver_summary = false, -- 是否记录求解器的摘要，如果为true，表示在控制台输出求解器的信息，如迭代次数，收敛情况等
    use_online_imu_extrinsics_in_3d = true, -- 是否在线优化IMU的外参，如果为true，表示在优化过程中，同时优化IMU相对于激光雷达的位姿
    fix_z_in_3d = false, -- 是否固定z轴，如果为true，表示在优化过程中，不优化z轴的平移和旋转
    ceres_solver_options = { -- ceres求解器的选项
      use_nonmonotonic_steps = false, -- 是否使用非单调步长，如果为true，表示在优化过程中，允许目标函数的值增加，可以跳出局部最优
      max_num_iterations = 50, -- 最大迭代次数，表示优化过程中，最多进行多少次迭代
      num_threads = 7, -- 线程数，表示优化过程中，使用多少个线程进行计算
    },
  },
  max_num_final_iterations = 200, -- 最大最终迭代次数，表示在全局优化阶段，最多进行多少次迭代
  global_sampling_ratio = 0.003, -- 全局采样比例，表示在全局优化阶段，从所有的子图中，按照一定的比例，随机选择一些子图进行优化
  log_residual_histograms = true, -- 是否记录残差直方图，如果为true，表示在优化过程中，输出每个残差项的分布情况，可以用于分析优化效果
  global_constraint_search_after_n_seconds = 10., -- 全局约束搜索的时间间隔，表示每隔多少秒，进行一次全局约束搜索，寻找可能的回环
  --  overlapping_submaps_trimmer_2d = { -- 重叠子图修剪器，用于删除重叠的子图，减少计算量
  --    fresh_submaps_count = 1, -- 新鲜子图的数量，表示在每个轨迹中，保留多少个最新的子图，不进行修剪
  --    min_covered_area = 2, -- 最小覆盖面积，表示在修剪过程中，保留多少平方米的覆盖面积，不进行修剪
  --    min_added_submaps_count = 5, -- 最小添加子图的数量，表示在每个轨迹中，至少添加多少个子图，不进行修剪
  --  },
}

