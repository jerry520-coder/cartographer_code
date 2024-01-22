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

include "map_builder_server.lua"

MAP_BUILDER_SERVER.map_builder.use_trajectory_builder_2d = true --启用了 Map Builder Server 中的 2D 轨迹构建器。这表示系统将使用2D轨迹构建器来构建地图，适用于2D SLAM应用场景。

return MAP_BUILDER_SERVER
