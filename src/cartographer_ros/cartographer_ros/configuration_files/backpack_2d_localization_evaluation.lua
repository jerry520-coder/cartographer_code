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

include "backpack_2d_localization.lua"

-- output map to base_link for evaluation
-- 设置输出地图到 base_link 以便评估
options.provide_odom_frame = false
POSE_GRAPH.optimization_problem.log_solver_summary = true -- 为优化问题的求解器打印摘要信息

-- fast localization
MAP_BUILDER.num_background_threads = 12 -- 设置后台线程的数量
POSE_GRAPH.constraint_builder.sampling_ratio = 0.5 * POSE_GRAPH.constraint_builder.sampling_ratio --设置了约束生成的采样比率
POSE_GRAPH.global_sampling_ratio = 0.1 * POSE_GRAPH.global_sampling_ratio
POSE_GRAPH.max_num_final_iterations = 1 -- 设置了最大的优化迭代次数

return options
