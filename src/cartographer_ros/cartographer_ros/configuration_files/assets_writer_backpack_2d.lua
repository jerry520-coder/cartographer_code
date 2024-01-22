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
VOXEL_SIZE = 5e-2

include "transform.lua"

options = {
  tracking_frame = "base_link",
  pipeline = {
    -- 1. 进行最小和最大测距过滤，将距离在 1 到 60 米之外的点过滤掉
    {
      action = "min_max_range_filter",
      min_range = 1.,
      max_range = 60.,
    },
    -- 2. 在控制台上打印点云的点数
    {
      action = "dump_num_points",
    },

    -- Gray X-Rays. These only use geometry to color pixels.
    {
      action = "write_xray_image",
      voxel_size = VOXEL_SIZE,
      filename = "xray_yz_all",
      transform = YZ_TRANSFORM,
    },
    {
      action = "write_xray_image",
      voxel_size = VOXEL_SIZE,
      filename = "xray_xy_all",
      transform = XY_TRANSFORM,
    },
    {
      action = "write_xray_image",
      voxel_size = VOXEL_SIZE,
      filename = "xray_xz_all",
      transform = XZ_TRANSFORM,
    },

    -- We now use the intensities to color our points. We apply a linear
    -- transform to clamp our intensity values into [0, 255] and then use this
    -- value for RGB of our points. Every stage in the pipeline after this now
    -- receives colored points.
    --
    -- We write xrays again. These now use geometry and the intensities to
    -- color pixels - they look quite similar, just a little lighter.

-- 我们现在使用强度信息为点云上色。我们应用线性变换将我们的强度值压缩到 [0, 255] 范围，然后使用这个值作为我们点云的 RGB 值。
-- 管道中的每个阶段现在都接收到了彩色的点云。

-- 我们再次生成 X-ray 影像。这次使用几何信息和强度信息为像素上色。它们看起来相似，只是稍微更亮。

 -- 4. 使用强度信息为点云上色，将强度值映射到 [0, 255] 的范围内
    {
      action = "intensity_to_color",
      min_intensity = 0.,
      max_intensity = 4095.,
    },

    -- 5. 重新生成 X-ray 影像，这次使用几何和强度信息来给像素上色
    {
      action = "write_xray_image",
      voxel_size = VOXEL_SIZE,
      filename = "xray_yz_all_intensity",
      transform = YZ_TRANSFORM,
    },
    {
      action = "write_xray_image",
      voxel_size = VOXEL_SIZE,
      filename = "xray_xy_all_intensity",
      transform = XY_TRANSFORM,
    },
    {
      action = "write_xray_image",
      voxel_size = VOXEL_SIZE,
      filename = "xray_xz_all_intensity",
      transform = XZ_TRANSFORM,
    },

    -- We also write a PLY file at this stage, because gray points look good.
    -- The points in the PLY can be visualized using
    -- https://github.com/cartographer-project/point_cloud_viewer.
     -- 6. 写入 PLY 文件，包含灰度信息的点云，可用于可视化
    {
      action = "write_ply",
      filename = "points.ply",
    },

    -- Now we recolor our points by frame and write another batch of X-Rays. It
    -- is visible in them what was seen by the horizontal and the vertical
    -- laser.
     -- 7. 根据激光束的方向为点云上色，并生成另一批 X-Ray 影像
    {
      action = "color_points",
      frame_id = "horizontal_laser_link",
      color = { 255., 0., 0. },
    },
    {
      action = "color_points",
      frame_id = "vertical_laser_link",
      color = { 0., 255., 0. },
    },

    {
      action = "write_xray_image",
      voxel_size = VOXEL_SIZE,
      filename = "xray_yz_all_color",
      transform = YZ_TRANSFORM,
    },
    {
      action = "write_xray_image",
      voxel_size = VOXEL_SIZE,
      filename = "xray_xy_all_color",
      transform = XY_TRANSFORM,
    },
    {
      action = "write_xray_image",
      voxel_size = VOXEL_SIZE,
      filename = "xray_xz_all_color",
      transform = XZ_TRANSFORM,
    },
  }
}

return options
