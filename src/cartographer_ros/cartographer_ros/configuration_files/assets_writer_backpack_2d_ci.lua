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

-- WARNING: we create a lot of X-Rays of a potentially large space in this
-- pipeline. For example, running over the
-- cartographer_paper_deutsches_museum.bag requires ~25GiB of memory. You can
-- reduce this by writing fewer X-Rays or upping VOXEL_SIZE - which is the size
-- of a pixel in a X-Ray.

-- 设置体素大小（Voxel Size），即地图中每个体素的尺寸
VOXEL_SIZE = 5e-2

-- 包含一个名为 "transform.lua" 的外部 Lua 文件，用于定义坐标变换的一些函数
include "transform.lua"

options = {
  -- 设置用于 SLAM 的坐标系
  tracking_frame = "base_link",
   -- 设置 SLAM 管道，包含一系列操作
  pipeline = {
    {
       -- 进行最小和最大测距过滤，将距离在 1 到 60 米之外的点过滤掉
      action = "min_max_range_filter",
      min_range = 1.,
      max_range = 60.,
    },
     -- 在控制台上打印点云的点数
    {
      action = "dump_num_points",
    },
    -- 在 YZ 平面上生成 X-ray 影像，并保存为图像文件
    {
      action = "write_xray_image",
      voxel_size = VOXEL_SIZE,
      filename = "xray_yz_all",
      transform = YZ_TRANSFORM,-- 使用之前定义的坐标变换
    },
     -- 在 XY 平面上生成 X-ray 影像，并保存为图像文件
    {
      action = "write_xray_image",
      voxel_size = VOXEL_SIZE,
      filename = "xray_xy_all",
      transform = XY_TRANSFORM,
    },
     -- 在 XZ 平面上生成 X-ray 影像，并保存为图像文件
    {
      action = "write_xray_image",
      voxel_size = VOXEL_SIZE,
      filename = "xray_xz_all",
      transform = XZ_TRANSFORM,
    },
  }
}

return options
