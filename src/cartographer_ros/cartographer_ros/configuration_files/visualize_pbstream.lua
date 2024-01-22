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

-- 引入一个外部 Lua 文件 "backpack_2d.lua"，该文件包含了一些与2D SLAM相关的设置
include "backpack_2d.lua"

-- 设置 POSE_GRAPH 约束生成器的采样比率为 0，即不使用约束
POSE_GRAPH.constraint_builder.sampling_ratio = 0

-- 设置 POSE_GRAPH 全局采样比率为 0，即不进行全局优化
POSE_GRAPH.global_sampling_ratio = 0

-- 设置 POSE_GRAPH 每多少节点进行一次优化，这里设为 0 表示禁用自动优化
POSE_GRAPH.optimize_every_n_nodes = 0

-- 返回配置选项
return options

