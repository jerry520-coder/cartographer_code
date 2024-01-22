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

include "trajectory_builder_2d.lua"
include "trajectory_builder_3d.lua"

-- 定义 TRAJECTORY_BUILDER 表，包含了 2D 和 3D 轨迹构建器以及一些全局设置
TRAJECTORY_BUILDER = {
  trajectory_builder_2d = TRAJECTORY_BUILDER_2D,
  trajectory_builder_3d = TRAJECTORY_BUILDER_3D,

  -- 用于纯定位（pure localization）的裁剪器设置，被注释掉表示未启用
--  pure_localization_trimmer = {
--    max_submaps_to_keep = 3,
--  },
  collate_fixed_frame = true, --是否通过固定帧修正
  collate_landmarks = false, --是否通过反光板修正
}
