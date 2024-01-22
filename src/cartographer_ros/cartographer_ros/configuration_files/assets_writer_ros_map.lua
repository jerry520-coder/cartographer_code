-- Copyright 2018 The Cartographer Authors
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

options = {
  tracking_frame = "base_link",
  pipeline = {
    -- 1. 进行最小和最大测距过滤，将距离在 1 到 60 米之外的点过滤掉
    {
      action = "min_max_range_filter",
      min_range = 1.,
      max_range = 60.,
    },
    {
      action = "write_ros_map",
      range_data_inserter = {
        insert_free_space = true,-- 插入自由空间信息
        hit_probability = 0.55,-- 激光束击中的概率
        miss_probability = 0.49, -- 激光束未击中的概率
      },
      filestem = "map", -- 地图文件的文件名前缀
      resolution = 0.05, -- 地图分辨率
    }
  }
}

return options
