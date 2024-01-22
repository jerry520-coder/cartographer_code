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

-- 引入pose_graph.lua文件，它定义了POSE_GRAPH对象，用来设置位姿图优化的选项
include "pose_graph.lua"

-- 定义一个名为MAP_BUILDER的表，用来存储地图构建的参数和选项
MAP_BUILDER = {
  -- 设置是否使用2D的轨迹构建器，如果为false，不会使用TRAJECTORY_BUILDER_2D对象
  use_trajectory_builder_2d = false,
  -- 设置是否使用3D的轨迹构建器，如果为false，不会使用TRAJECTORY_BUILDER_3D对象
  use_trajectory_builder_3d = false,
  -- 设置后台处理的线程数，值越大，处理能力越强，但消耗资源越多
  num_background_threads = 4,
  -- 设置位姿图优化的选项，它是一个POSE_GRAPH对象
  pose_graph = POSE_GRAPH,
  -- 设置是否按照轨迹进行整合，如果为false，会按照时间顺序整合
  collate_by_trajectory = false,
}
