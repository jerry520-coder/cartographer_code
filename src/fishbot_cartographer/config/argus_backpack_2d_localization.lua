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

include "argus_backpack_2d.lua"

-- 配置纯定位（pure localization）修剪器，限制保留的子图数量
TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3, --3限制了保留的子图数量，这对于纯定位（不进行地图构建，只进行定位）场景可能很有用 (默认 3)
}
POSE_GRAPH.optimize_every_n_nodes = 20 -- 配置了优化的频率。这表示系统将在每构建20个节点之后执行一次图优化，以提高定位和地图的精度

return options
